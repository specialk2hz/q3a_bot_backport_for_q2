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

/* Called once the AAS file is loaded, before the routing is initialised. */
void IG_AdjustGrappleCosts(void)
{
	int i, n = 0;
	float start, t;
	vec3_t d;
	aas_reachability_t *r;

	start = LibVarGetValue("ig_rs_startgrapple");
	if (start <= 0) return;
	for (i = 0; i < aasworld.reachabilitysize; i++)
	{
		r = &aasworld.reachability[i];
		if ((r->traveltype & TRAVELTYPE_MASK) != TRAVEL_GRAPPLEHOOK) continue;
		/* be_aas_reach.c's formula, with the fixed part ours */
		VectorSubtract(r->end, r->start, d);
		t = start + VectorLength(d) * 0.25f;
		r->traveltime = t > 65535 ? 65535 : (unsigned short) t;
		n++;
	}
	if (n) botimport.Print(PRT_MESSAGE, "ig: %d grapple reachabilities at start cost %d\n", n, (int) start);
}
