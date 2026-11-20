/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_hook.c -- the Q3 bots' grapple, wired to Quake II offhand hooks.
 *
 * Instagib fork (specialk2hz/q3a_bot_backport_for_q2), IG-6 in the Colosseum
 * fork's IG-SPECS.md. Upstream files reach this file only through one-line
 * calls marked "IG:". Every function here is a no-op unless the game turns
 * the feature on, so off is upstream behaviour.
 *
 * Q3's grapple travel (be_ai_move.c, BotTravel_Grapple) already knows an
 * offhand hook: it walks to the reachability's start, looks at its end,
 * issues cmd_grappleon, and releases with cmd_grappleoff when the pull stops
 * gaining ground. What it cannot do on Quake II is SEE the hook:
 * GrappleState() looks for an ET_MISSILE entity whose weapon is
 * weapindex_grapple, and the adapter makes neither -- OSP's hook is a beam
 * (RF_BEAM) with no projectile effects, Threewave's a model with none, so
 * both are ET_GENERAL. And GrappleState() never asks whose hook it is, which
 * with five bots means any bot's hook.
 *
 * Libvars, all set by the game (Colosseum fork, src/ig/ig_bot.c):
 *   ig_hookdetect       1: classify Q2 hook entities, and judge the grapple
 *                       state by the bot's own hook (IG_GrappleState)
 *   ig_rs_startgrapple  >0: the fixed cost of a grapple reachability,
 *                       applied to the loaded AAS file in place of the one
 *                       bspc baked in (rs_startgrapple, 500 = 5 s)
 * and Q3's own, which the game sets alongside: bot_grapple, offhandgrapple,
 * cmd_grappleon, cmd_grappleoff.
 */

#include "../game_q3/q_shared.h"
#include "l_memory.h"
#include "l_libvar.h"
#include "l_utils.h"
#include "l_script.h"
#include "l_precomp.h"
#include "l_struct.h"
#include "l_log.h"
#include "aasfile.h"
#include "../game_q3/botlib.h"
#include "../game_q3/be_aas.h"
#include "be_aas_funcs.h"
#include "be_interface.h"
#include "be_aas_def.h"
#include "../game_q3/be_ea.h"
#include "be_ig_hook.h"

#define Q2_RF_BEAM          0x80
#define Q2_SOLID_BBOX       2
#define Q2_HOOK_MODEL       "models/weapons/grapple/hook/tris.md2"

/* How far a hook may lie from the line between the bot's eye and the anchor
 * and still be taken for the bot's own. A hook in flight is on that line by
 * construction; another bot's hook almost never is. */
#define IG_HOOK_LINE_DIST   64
/* A hook this close to the anchor, and not moving, is attached there. */
#define IG_HOOK_ANCHOR_DIST 48

int IG_HookDetect(void)
{
	return LibVarGetValue("ig_hookdetect") != 0;
}

/* The adapter's model table lookup for the Threewave hook, cached per map. */
static int IG_HookModelIndex(char **names, int count)
{
	int i;

	for (i = 1; names && i < count; i++)
	{
		if (names[i] && !Q_stricmp(names[i], Q2_HOOK_MODEL)) return i;
	}
	return -1;
}

int IG_IsHookEntity(int solid, int renderfx, int modelindex, char **names, int count)
{
	if (!IG_HookDetect()) return false;
	if (solid != Q2_SOLID_BBOX) return false;
	/* OSP's laser hook (and Lithium's): a beam that is solid. target_laser
	 * is a beam too, but SOLID_NOT. */
	if (renderfx & Q2_RF_BEAM) return true;
	/* Threewave's cable hook: its own model */
	return modelindex > 0 && modelindex == IG_HookModelIndex(names, count);
}

/* Distance from p to the segment a-b. */
static float IG_SegmentDist(const vec3_t p, const vec3_t a, const vec3_t b)
{
	vec3_t ab, ap, c;
	float t, len2;

	VectorSubtract(b, a, ab);
	VectorSubtract(p, a, ap);
	len2 = DotProduct(ab, ab);
	t = len2 > 0 ? DotProduct(ap, ab) / len2 : 0;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	VectorMA(a, t, ab, c);
	VectorSubtract(p, c, c);
	return VectorLength(c);
}

/* GrappleState() for Q2 hooks: 0 no hook of ours, 1 ours in flight (or
 * anywhere on the line), 2 ours attached at the anchor. */
int IG_GrappleState(const vec3_t origin, const vec3_t viewoffset, const vec3_t anchor)
{
	int i;
	vec3_t eye, d;
	aas_entityinfo_t entinfo;

	VectorAdd(origin, viewoffset, eye);
	for (i = AAS_NextEntity(0); i; i = AAS_NextEntity(i))
	{
		if (AAS_EntityType(i) != 3 /* ET_MISSILE */) continue;
		AAS_EntityInfo(i, &entinfo);
		if (entinfo.weapon != IG_GRAPPLE_WEAPON) continue;
		if (IG_SegmentDist(entinfo.origin, eye, anchor) > IG_HOOK_LINE_DIST) continue;
		VectorSubtract(entinfo.origin, anchor, d);
		if (VectorLength(d) < IG_HOOK_ANCHOR_DIST &&
			VectorCompare(entinfo.origin, entinfo.lastvisorigin))
			return 2;
		return 1;
	}
	return 0;
}

/* BotTravel_Grapple's distance to the anchor, for a Q2 hook. Q3 measures it
 * horizontally, because its hook pulls a player into a wall; an OSP hook pulls
 * toward the anchor in 3D, so on a high or ceiling anchor the horizontal
 * distance stops shrinking while the bot is still rising, and Q3 let go
 * mid-climb. In 3D, from the eye as OSP measures it: OSP hangs the player
 * once the eye is within 64 units, and Q3 releases inside 48 when it stops
 * getting closer, so the hang reads as 64 - 24 = 40 and releases there. */
#define IG_HANG_OFFSET 24
float IG_GrappleDist(const vec3_t origin, const vec3_t viewoffset, const vec3_t anchor)
{
	vec3_t eye, d;
	float dist;

	VectorAdd(origin, viewoffset, eye);
	VectorSubtract(anchor, eye, d);
	dist = VectorLength(d) - IG_HANG_OFFSET;
	return dist < 0 ? 0 : dist;
}

/* Where to aim the hook. bspc proved a grapple route by sweeping the
 * player's box with its ORIGIN on the line from the start to the anchor
 * (AAS_Reachability_Grapple); OSP's hook pulls the EYE to the anchor
 * (GrapplePull), so the body rides a viewheight lower than that line and on a
 * shallow approach its feet catch the lip of the ledge: no more progress, Q3
 * lets go, the bot drops back. Aiming the same height higher on the wall puts
 * the body back on bspc's line. Only if that point is on the same surface (a
 * trace from the eye ends within 16 units of it); otherwise the anchor as is.
 * ig_hookaim 0 turns it off (default 1). */
void IG_GrappleAim(const vec3_t eye, const vec3_t anchor, float lift, int passent, vec3_t viewdir)
{
	vec3_t target, d;
	bsp_trace_t trace;

	if (!LibVarValue("ig_hookaim", "1")) return;
	VectorCopy(anchor, target);
	target[2] += lift;
	trace = AAS_Trace((vec_t *) eye, NULL, NULL, target, passent, CONTENTS_SOLID);
	VectorSubtract(target, trace.endpos, d);
	if (VectorLength(d) > 16) return;
	VectorSubtract(target, eye, viewdir);
}

/* The watchdog. Q3's grapple travel only runs while the bot follows its
 * route (BotMoveToGoal). A bot that sees an enemy mid-hook switches to its
 * fight movement and nothing lets go: on an OSP hook it hangs there for
 * hook_holdtime (7.5 s), a free kill. So BotTravel_Grapple notes every think
 * it runs, and once per frame, for every bot, a hook whose travel has not run
 * for IG_HOOK_ORPHAN seconds is let go of. */
#define IG_HOOK_ORPHAN  0.3f
static float ig_grapplenote[MAX_CLIENTS];

void IG_GrappleTravelNote(int client)
{
	if (client >= 0 && client < MAX_CLIENTS) ig_grapplenote[client] = AAS_Time();
}

void IG_GrappleWatchdog(int client)
{
	if (!IG_HookDetect() || client < 0 || client >= MAX_CLIENTS) return;
	if (ig_grapplenote[client] <= 0) return;
	/* a new map restarts the clock */
	if (AAS_Time() < ig_grapplenote[client]) { ig_grapplenote[client] = 0; return; }
	if (AAS_Time() - ig_grapplenote[client] < IG_HOOK_ORPHAN) return;
	EA_Command(client, LibVarGetString("cmd_grappleoff"));
	ig_grapplenote[client] = 0;
}

#define IG_FEET  24   /* a reachability's start is the origin; feet are 24 below */
#define IG_CLIMB 48   /* a climb lands at least this far above the start's feet */

/* Called once the AAS file is loaded, before the routing is initialised. */
void IG_AdjustGrappleCosts(void)
{
	int i, n = 0, nodrop = 0, climbonly;
	float start, t;
	vec3_t d;
	aas_reachability_t *r;

	start = LibVarGetValue("ig_rs_startgrapple");
	if (start <= 0) return;
	climbonly = LibVarValue("ig_hookclimbonly", "1") != 0;
	for (i = 0; i < aasworld.reachabilitysize; i++)
	{
		r = &aasworld.reachability[i];
		if ((r->traveltype & TRAVELTYPE_MASK) != TRAVEL_GRAPPLEHOOK) continue;
		/* Climbs only: bspc also makes routes that hook a ceiling and drop to
		 * the floor below it, often no higher than the start -- a fast way
		 * across a room that leaves the bot falling. A route whose landing
		 * floor is not IG_CLIMB units above the start is taken out: made
		 * TRAVEL_INVALID, whose TFL_INVALID no route allows (be_aas_route.c
		 * skips it). Not a huge cost: route times are summed in an
		 * unsigned short and would wrap. */
		if (climbonly && aasworld.areas[r->areanum].mins[2] < r->start[2] - IG_FEET + IG_CLIMB)
		{
			r->traveltype = (r->traveltype & ~TRAVELTYPE_MASK) | TRAVEL_INVALID;
			nodrop++;
			continue;
		}
		/* be_aas_reach.c's formula, with the fixed part ours */
		VectorSubtract(r->end, r->start, d);
		t = start + VectorLength(d) * 0.25f;
		r->traveltime = t > 65535 ? 65535 : (unsigned short) t;
		n++;
	}
	if (n || nodrop) botimport.Print(PRT_MESSAGE, "ig: %d grapple reachabilities at start cost %d, %d that climb less than %d taken out\n", n, (int) start, nodrop, IG_CLIMB);
}
