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
#include <stdlib.h>

void AAS_RemoveRoutingCacheUsingArea(int areanum);    /* be_aas_route.c */

#define Q2_RF_BEAM          0x80
#define Q2_SOLID_NOT        0
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
	/* OSP's laser hook (and Lithium's) is a beam: SOLID_BBOX while it flies,
	 * SOLID_NOT once it has latched on (GrappleTouch). A target_laser is a
	 * beam too and is not told apart here: IG_GrappleState only takes a hook
	 * that lies on the bot's own line to its anchor, which a map's laser
	 * practically never does. */
	if ((renderfx & Q2_RF_BEAM) && (solid == Q2_SOLID_BBOX || solid == Q2_SOLID_NOT)) return true;
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

/* BotTravel_Grapple's "distance to the anchor", which is what decides when
 * the bot lets go: inside 48 and no closer than last frame, it releases; not
 * gaining 2 units a frame for 0.4 s, it gives up and releases (Q3's rules).
 *
 * Q3 measured it horizontally, for a hook that pulls a player into a wall.
 * An OSP hook pulls the eye to the anchor in 3D, and where the anchor is on
 * the back wall of a ledge the bot arrives fast, beside the ledge rather than
 * over it, and let go there it slid off and fell -- below where it started.
 * So the value returned is a steering signal, not a measurement:
 *   - over the landing area (the reachability's area): 0, so Q3 releases on
 *     the next frame and the bot drops onto the ledge;
 *   - otherwise, for up to IG_HOLD_MAX seconds after the hook went out, a
 *     value that keeps shrinking but stays above 48: Q3 neither releases nor
 *     gives up, and the pull, then OSP's hang, carries the bot onto it;
 *   - after that, the eye's real distance to the anchor, so Q3's give-up
 *     rule ends a hook that is not getting there. */
#define IG_HOLD_MAX  1.5f
static float ig_hookout[MAX_CLIENTS];

float IG_GrappleDist(int client, const vec3_t origin, const vec3_t viewoffset, const struct aas_reachability_s *reach, float last)
{
	vec3_t eye, d, below;
	float real;

	VectorAdd(origin, viewoffset, eye);
	VectorSubtract(reach->end, eye, d);
	real = VectorLength(d);
	if (client < 0 || client >= MAX_CLIENTS) return real;
	/* BotTravel_Grapple sets lastgrappledist to 999999 when it fires */
	if (last > 99999) ig_hookout[client] = AAS_Time();
	VectorCopy(origin, below);
	below[2] -= 48;
	if (AAS_PointAreaNum((vec_t *) origin) == reach->areanum || AAS_PointAreaNum(below) == reach->areanum)
		return 0;
	if (AAS_Time() - ig_hookout[client] < IG_HOLD_MAX)
	{
		if (last > 99999) return real > 100 ? real : 100;
		return last - 3 > 50 ? last - 3 : 50;
	}
	return real;
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

/* The walk to a grapple route's start. Q3 fires only within 5 units of it
 * and, inside 70, walks at 4 units/s per unit left (BotTravel_Grapple):
 * 40 at 10 units. Quake II's ground friction (stopspeed 100) eats input that
 * small, so the bot ran in place a few units short until the route timed
 * out. Here: within IG_START_ARRIVED counts as there (the hook's own aim is
 * from wherever the bot stands, and its path is traced before firing), and
 * the approach never asks for less than IG_START_MINDIST's speed (160). */
#define IG_START_ARRIVED 24
#define IG_START_MINDIST 40
float IG_GrappleStartDist(float dist)
{
	if (dist < IG_START_ARRIVED) return 0;
	return dist < IG_START_MINDIST ? IG_START_MINDIST : dist;
}

/* A grapple route whose hook path is blocked. BotTravel_Grapple traces the
 * line to the anchor before firing and reports a movement failure when it is
 * not clear -- and the decision code answers a failure by clearing the
 * avoided reachabilities (trap_BotResetAvoidReach, ai_dmnet.c), so the route
 * planner chose the same hook again: the bot stood looking up, failing,
 * forever. The trace is against the world only, so a blocked route stays
 * blocked: after IG_BLOCKED_MAX failures it is taken out for the rest of the
 * map (TRAVEL_INVALID, and the routing cache around it rebuilt). */
#define IG_BLOCKED_MAX 2
static unsigned char *ig_blocked;
static int ig_blockedsize;

void IG_GrappleBlocked(int reachnum)
{
	aas_reachability_t *r;

	if (!ig_blocked || reachnum <= 0 || reachnum >= ig_blockedsize) return;
	if (++ig_blocked[reachnum] < IG_BLOCKED_MAX) return;
	r = &aasworld.reachability[reachnum];
	if ((r->traveltype & TRAVELTYPE_MASK) != TRAVEL_GRAPPLEHOOK) return;
	r->traveltype = (r->traveltype & ~TRAVELTYPE_MASK) | TRAVEL_INVALID;
	AAS_RemoveRoutingCacheUsingArea(AAS_PointAreaNum(r->start));
	botimport.Print(PRT_MESSAGE, "ig: grapple reachability %d blocked, taken out\n", reachnum);
}

/* Called once the AAS file is loaded, before the routing is initialised. */
void IG_AdjustGrappleCosts(void)
{
	int i, n = 0, nodrop = 0, climbonly;
	float start, t;
	vec3_t d;
	aas_reachability_t *r;

	free(ig_blocked);   /* libc, not the botlib's pools: it outlives nothing */
	ig_blocked = NULL;
	ig_blockedsize = 0;
	start = LibVarGetValue("ig_rs_startgrapple");
	if (start <= 0) return;
	ig_blockedsize = aasworld.reachabilitysize;
	ig_blocked = calloc(ig_blockedsize, 1);
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
