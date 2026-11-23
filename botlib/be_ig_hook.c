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
#define Q2_LMCTF_HOOK_MODEL "models/objects/ghook/tris.md2"   /* hook_type lmctf (IG-34) */

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

/* Is this model index a hook's model: Threewave's, or LMCTF's (IG-34)? */
static int IG_HookModel(int modelindex, char **names, int count)
{
	if (!names || modelindex <= 0 || modelindex >= count || !names[modelindex]) return false;
	return !Q_stricmp(names[modelindex], Q2_HOOK_MODEL) || !Q_stricmp(names[modelindex], Q2_LMCTF_HOOK_MODEL);
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
	/* Threewave's cable hook, LMCTF's: their own models */
	return IG_HookModel(modelindex, names, count);
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
/* no-progress give-up: the best distance and when it last improved */
static float ig_hookbest[MAX_CLIENTS], ig_hookbesttime[MAX_CLIENTS];
#define IG_STUCK_TIME  0.6f
#define IG_STUCK_GAIN  8

/* The route a bot is hooking on, for the outcome check in the watchdog. */
static int   ig_hookreach[MAX_CLIENTS];   /* reachability of the hook now out, 0: none */
static float ig_hookseen[MAX_CLIENTS];    /* last frame it was active */
static int   ig_checkreach[MAX_CLIENTS];  /* a finished hook being checked */
static float ig_checkuntil[MAX_CLIENTS];

float IG_GrappleDist(int client, int reachnum, const vec3_t origin, const vec3_t viewoffset, const struct aas_reachability_s *reach, float last)
{
	vec3_t eye, d, below;
	float real;

	VectorAdd(origin, viewoffset, eye);
	VectorSubtract(reach->end, eye, d);
	real = VectorLength(d);
	if (client < 0 || client >= MAX_CLIENTS) return real;
	ig_hookreach[client] = reachnum;
	ig_hookseen[client] = AAS_Time();
	/* BotTravel_Grapple sets lastgrappledist to 999999 when it fires */
	if (last > 99999) {
		ig_hookout[client] = AAS_Time();
		ig_hookbest[client] = real;
		ig_hookbesttime[client] = AAS_Time();
	}
	VectorCopy(origin, below);
	below[2] -= 48;
	if (AAS_PointAreaNum((vec_t *) origin) == reach->areanum || AAS_PointAreaNum(below) == reach->areanum)
		return 0;
	if (real < ig_hookbest[client] - IG_STUCK_GAIN) {
		ig_hookbest[client] = real;
		ig_hookbesttime[client] = AAS_Time();
	}
	if (AAS_Time() - ig_hookout[client] < IG_HOLD_MAX)
	{
		if (last > 99999) return real > 100 ? real : 100;
		return last - 3 > 50 ? last - 3 : 50;
	}
	/* Past the hold and getting no closer: let go. Q3's own give-up waits for
	 * the hook to go out of sight, which a bot hanging at a visible anchor
	 * beside the ledge never does -- it hung until OSP's hook_holdtime (7.5 s;
	 * Razor, q2rdm11). 0 twice is BotTravel_Grapple's "normal end". */
	if (AAS_Time() - ig_hookbesttime[client] > IG_STUCK_TIME) {
		botimport.Print(PRT_MESSAGE, "ig: grapple stuck %d, let go\n", client);
		return 0;
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

/* IG-34: > 1 for a hook slower than OSP's 1650 u/s (the game passes the live
 * hook's speed in ig_hookfly): it is out longer before it bites. */
float IG_HookFlyScale(void)
{
	float v = LibVarValue("ig_hookfly", "0");
	return v > 100 ? 1650.0f / v : 1;
}

void IG_GrappleTravelNote(int client)
{
	if (client >= 0 && client < MAX_CLIENTS) ig_grapplenote[client] = AAS_Time();
}

static void IG_RouteFailed(int reachnum);

/* Did the hook deliver? From the frame a bot's hook stops being travelled,
 * it has IG_LAND_TIME seconds to be in the route's landing area. A bot that
 * dies meanwhile proves nothing either way. A route that fails twice is
 * taken out for the map (IG_RouteFailed): bspc cannot tell that OSP's hang,
 * up to 64 units out from the wall, leaves a narrow ledge out of reach. */
#define IG_LAND_TIME 1.5f
static void IG_HookOutcome(int client)
{
	aas_entityinfo_t e;
	int reach;

	if (ig_hookreach[client] && AAS_Time() - ig_hookseen[client] > 0.25f)
	{
		ig_checkreach[client] = ig_hookreach[client];
		ig_checkuntil[client] = AAS_Time() + IG_LAND_TIME;
		ig_hookreach[client] = 0;
	}
	reach = ig_checkreach[client];
	if (!reach) return;
	if (AAS_Time() > ig_checkuntil[client] + 5 || reach >= aasworld.reachabilitysize)
	{   /* a new map */
		ig_checkreach[client] = 0;
		return;
	}
	AAS_EntityInfo(client, &e);
	if (!e.valid || (e.flags & 1 /* EF_DEAD */))
	{
		ig_checkreach[client] = 0;
		return;
	}
	if (AAS_PointAreaNum(e.origin) == aasworld.reachability[reach].areanum)
	{
		ig_checkreach[client] = 0;
		return;
	}
	if (AAS_Time() > ig_checkuntil[client])
	{
		ig_checkreach[client] = 0;
		IG_RouteFailed(reach);
	}
}

void IG_GrappleWatchdog(int client)
{
	if (!IG_HookDetect() || client < 0 || client >= MAX_CLIENTS) return;
	IG_HookOutcome(client);
	if (ig_grapplenote[client] <= 0) return;
	/* a new map restarts the clock */
	if (AAS_Time() < ig_grapplenote[client]) { ig_grapplenote[client] = 0; return; }
	if (AAS_Time() - ig_grapplenote[client] < IG_HOOK_ORPHAN * IG_HookFlyScale()) return;   /* IG-34 */
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

/* IG-32: climbs on the run (libvar ig_climbrun, default 0). Billy fires a
 * climb running (~300 u/s); a bot walked to the route's start, stopped, and
 * stood until its aim was inside 2 degrees -- 60% of its climbs left from a
 * standstill. With this, near the start (IG_RUN_FIRE) it keeps running and
 * fires once its view is within IG_RUN_AIM degrees of the aim (the hook's
 * path is traced before it fires); and it lets go as soon as its feet are over the landing's
 * height and it is near the landing, so the momentum carries it over the
 * lip instead of riding the pull to the anchor and hanging there. */
#define IG_RUN_FIRE  40
#define IG_RUN_AIM   6    /* degrees, at most */
#define IG_RUN_MISS  16   /* units off the aim point the hook may land */
#define IG_RUN_LAND  48   /* from the landing area, horizontally */
#define IG_RUN_CLEAR 0.8f /* of the way to the anchor the body must clear */
/* bit 1: fire on the run; bit 2: let go over the landing. 3 = both */
int IG_ClimbRun(void)
{
	return (int) LibVarValue("ig_climbrun", "0");
}

int IG_ClimbFireNow(const vec3_t origin, const vec3_t viewoffset, const vec3_t viewangles, const vec3_t start, const vec3_t aimdir, int passent)
{
	vec3_t d, ideal, eye, end;
	aas_trace_t trace;
	float len, tol;

	if (!(IG_ClimbRun() & 1)) return 0;
	VectorSubtract(start, origin, d);
	d[2] = 0;
	if (VectorLength(d) > IG_RUN_FIRE) return 0;
	/* the hook must land within IG_RUN_MISS of the aim point, or the pull
	 * takes another line: about 3 degrees at a usual 300 units */
	len = VectorLength(aimdir);
	if (len < 1) return 0;
	tol = RAD2DEG(atan(IG_RUN_MISS / len));
	if (tol < 2) tol = 2;
	if (tol > IG_RUN_AIM) tol = IG_RUN_AIM;
	vectoangles(aimdir, ideal);
	if (fabs(AngleMod(ideal[0] - viewangles[0] + 180) - 180) >= tol ||
		fabs(AngleMod(ideal[1] - viewangles[1] + 180) - 180) >= tol) return 0;
	/* from here, not bspc's start, the pull carries the body along a lower
	 * line: only if the body clears it (to IG_RUN_CLEAR of the way) */
	VectorAdd(origin, viewoffset, eye);
	VectorMA(origin, IG_RUN_CLEAR, aimdir, end);
	trace = AAS_TraceClientBBox((vec_t *) origin, end, PRESENCE_NORMAL, passent);
	return trace.fraction >= 1;
}

int IG_ClimbLetGo(const vec3_t origin, const struct aas_reachability_s *reach)
{
	vec3_t d;
	aas_area_t *area;
	int i;

	if (!(IG_ClimbRun() & 2) || reach->areanum <= 0 || reach->areanum >= aasworld.numareas) return 0;
	area = &aasworld.areas[reach->areanum];
	/* the area is in origin space: over its bottom, the feet clear its floor */
	if (origin[2] < area->mins[2] + 8) return 0;
	for (i = 0; i < 2; i++)
		d[i] = origin[i] < area->mins[i] ? area->mins[i] - origin[i] : origin[i] > area->maxs[i] ? origin[i] - area->maxs[i] : 0;
	return d[0] * d[0] + d[1] * d[1] <= IG_RUN_LAND * IG_RUN_LAND;
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

static void IG_TakeOut(int reachnum, const char *why)
{
	aas_reachability_t *r;

	if (!ig_blocked || reachnum <= 0 || reachnum >= ig_blockedsize) return;
	if (++ig_blocked[reachnum] < IG_BLOCKED_MAX) return;
	r = &aasworld.reachability[reachnum];
	if ((r->traveltype & TRAVELTYPE_MASK) != TRAVEL_GRAPPLEHOOK) return;
	r->traveltype = (r->traveltype & ~TRAVELTYPE_MASK) | TRAVEL_INVALID;
	AAS_RemoveRoutingCacheUsingArea(AAS_PointAreaNum(r->start));
	botimport.Print(PRT_MESSAGE, "ig: grapple reachability %d %s, taken out\n", reachnum, why);
}

void IG_GrappleBlocked(int reachnum)
{
	IG_TakeOut(reachnum, "blocked");
}

static void IG_RouteFailed(int reachnum)
{
	IG_TakeOut(reachnum, "did not land");
}

/* A hook near `point`: 0 none, 1 one in flight, 2 one latched (not moving),
 * its position in hookpos. For a slingshot (game_q3/ai_ig_sling.c), whose hook
 * lands near, not on, the anchor it was aimed at. */
int IG_HookNear(const vec3_t point, float radius, vec3_t hookpos)
{
	int i, found = 0;
	vec3_t d;
	aas_entityinfo_t entinfo;

	for (i = AAS_NextEntity(0); i; i = AAS_NextEntity(i))
	{
		if (AAS_EntityType(i) != 3 /* ET_MISSILE */) continue;
		AAS_EntityInfo(i, &entinfo);
		if (entinfo.weapon != IG_GRAPPLE_WEAPON) continue;
		VectorSubtract(entinfo.origin, point, d);
		if (VectorLength(d) > radius) continue;
		VectorCopy(entinfo.origin, hookpos);
		if (VectorCompare(entinfo.origin, entinfo.lastvisorigin)) return 2;
		found = 1;
	}
	return found;
}

/* One clock for every hook a bot fires -- route hooks, slingshots, combat
 * hooks, the lava escape: OSP refuses a hook within hook_wait (0.5 s) of the
 * previous FIRE (OSP_hookon_cmd), and the techniques, each timing itself,
 * spent attempts on each other's wait. */
#define IG_HOOK_WAIT 0.55f
static float ig_lastfire[MAX_CLIENTS];

void IG_NoteHookFire(int client)
{
	if (client >= 0 && client < MAX_CLIENTS) ig_lastfire[client] = AAS_Time();
}

int IG_CanFireHook(int client)
{
	if (client < 0 || client >= MAX_CLIENTS) return 0;
	if (ig_lastfire[client] > AAS_Time()) ig_lastfire[client] = 0;      /* a new map */
	return AAS_Time() - ig_lastfire[client] >= IG_HOOK_WAIT;
}

/* The map's high ground, for the roam goal (game_q3/ai_ig_roam.c, IG-7):
 * standable areas whose floor is in the top 40% of the map's floor heights
 * (5th to 95th percentile), highest first, at least IG_HG_SPREAD apart, at
 * most IG_HG_MAX. Worked out once per map, on first use. */
#define IG_HG_MAX     16
#define IG_HG_SPREAD  400
#define IG_HG_FRAC    0.6f
static int    ig_hgcount = -1;      /* -1: not worked out for this map */
static int    ig_hgarea[IG_HG_MAX];
static vec3_t ig_hgpoint[IG_HG_MAX];

static int IG_FloatCmp(const void *a, const void *b)
{
	float d = *(const float *)a - *(const float *)b;
	return d < 0 ? -1 : d > 0;
}

static void IG_FindHighGround(void)
{
	int i, j, n = 0, *cand;
	float *floors, *sorted, zlo, zhi, cut;
	vec3_t d;
	aas_area_t *a;
	aas_areasettings_t *st;

	ig_hgcount = 0;
	if (aasworld.numareas < 2) return;
	floors = calloc(aasworld.numareas, sizeof(float));
	sorted = calloc(aasworld.numareas, sizeof(float));
	cand = calloc(aasworld.numareas, sizeof(int));
	if (!floors || !sorted || !cand) { free(floors); free(sorted); free(cand); return; }
	for (i = 1; i < aasworld.numareas; i++)
	{
		a = &aasworld.areas[i];
		st = &aasworld.areasettings[i];
		if (!(st->areaflags & AREA_GROUNDED)) continue;
		if (!(st->presencetype & PRESENCE_NORMAL)) continue;
		if (st->contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_WATER |
							AREACONTENTS_JUMPPAD | AREACONTENTS_TELEPORTER)) continue;
		if (st->numreachableareas <= 0) continue;
		if (a->maxs[0] - a->mins[0] < 32 || a->maxs[1] - a->mins[1] < 32) continue;
		floors[i] = a->mins[2];
		sorted[n] = a->mins[2];
		cand[n++] = i;
	}
	if (n < 8) { free(floors); free(sorted); free(cand); return; }
	qsort(sorted, n, sizeof(float), IG_FloatCmp);
	zlo = sorted[n / 20];
	zhi = sorted[n - 1 - n / 20];
	cut = zlo + IG_HG_FRAC * (zhi - zlo);
	/* highest first: a selection by height over the candidates */
	for (;;)
	{
		int best = -1;

		for (i = 0; i < n; i++)
		{
			if (cand[i] <= 0 || floors[cand[i]] < cut) continue;
			if (best < 0 || floors[cand[i]] > floors[cand[best]]) best = i;
		}
		if (best < 0 || ig_hgcount >= IG_HG_MAX) break;
		a = &aasworld.areas[cand[best]];
		for (j = 0; j < ig_hgcount; j++)
		{
			VectorSubtract(ig_hgpoint[j], a->center, d);
			if (VectorLength(d) < IG_HG_SPREAD) break;
		}
		if (j == ig_hgcount)
		{
			ig_hgarea[ig_hgcount] = cand[best];
			VectorCopy(a->center, ig_hgpoint[ig_hgcount]);
			ig_hgpoint[ig_hgcount][2] = a->mins[2] + 24;   /* where a player stands */
			ig_hgcount++;
		}
		cand[best] = -cand[best];
	}
	free(floors); free(sorted); free(cand);
	botimport.Print(PRT_MESSAGE, "ig: %d high ground points (floor above %.0f)\n", ig_hgcount, cut);
}

int IG_HighGround(int max, int *areas, vec3_t *points)
{
	int i;

	if (ig_hgcount < 0) IG_FindHighGround();
	for (i = 0; i < ig_hgcount && i < max; i++)
	{
		areas[i] = ig_hgarea[i];
		VectorCopy(ig_hgpoint[i], points[i]);
	}
	return i;
}

/* Called once the AAS file is loaded, before the routing is initialised. */
void IG_AdjustGrappleCosts(void)
{
	int i, n = 0, nodrop = 0, climbonly;
	float start, t;
	vec3_t d;
	aas_reachability_t *r;

	ig_hgcount = -1;    /* a new map: its high ground is worked out on first use */
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

/* The landing spot of the jump pad a bot is flying, noted each frame by
 * BotFinishTravel_JumpPad (be_ai_move.c), for the pad hook
 * (game_q3/ai_ig_sling.c): hook ahead to the landing instead of riding the arc. */
static vec3_t ig_padend[MAX_CLIENTS];
static float  ig_padseen[MAX_CLIENTS];

void IG_NotePadFlight(int client, const vec3_t end)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	VectorCopy(end, ig_padend[client]);
	ig_padseen[client] = AAS_Time();
}

/* true while the bot is in a pad's flight (noted within the last 0.25 s) */
int IG_PadFlight(int client, vec3_t end)
{
	float t;

	if (client < 0 || client >= MAX_CLIENTS) return 0;
	t = AAS_Time() - ig_padseen[client];
	if (ig_padseen[client] <= 0 || t < 0 || t > 0.25f) return 0;
	VectorCopy(ig_padend[client], end);
	return 1;
}
