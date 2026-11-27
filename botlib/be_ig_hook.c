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

int IG_OwnHook(const vec3_t eye, float speed, vec3_t hookpos)
{
	int i, best = 0;
	float bestd = 40 + speed * 0.1f, d;
	vec3_t v;
	aas_entityinfo_t entinfo, found;

	for (i = AAS_NextEntity(0); i; i = AAS_NextEntity(i))
	{
		if (AAS_EntityType(i) != 3 /* ET_MISSILE */) continue;
		AAS_EntityInfo(i, &entinfo);
		if (entinfo.weapon != IG_GRAPPLE_WEAPON) continue;
		VectorSubtract(entinfo.old_origin, eye, v);
		d = VectorLength(v);
		if (d < bestd) { bestd = d; best = i; found = entinfo; }
	}
	if (!best) return 0;
	if (hookpos) VectorCopy(found.origin, hookpos);
	return VectorCompare(found.origin, found.lastvisorigin) ? 2 : 1;
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

/* IG-32 (2026-11-27, Billy: bots "completely stop moving and grapple and pull
 * to move when a seasoned player would be moving in the direction that they
 * want to grapple"): the run-in window -- within IG_RUN_FIRE of the route's
 * start, or up to IG_RUN_PAST beyond it along the way to the anchor and
 * within IG_RUN_SIDE of that line. A bot running in fires anywhere in it, and
 * keeps the climb route while it is in it (it used to drop the route the
 * moment the run took it into the next area: a third fewer climbs). */
#define IG_RUN_PAST  160
#define IG_RUN_SIDE  48
#define IG_RUN_EARLY 200
int IG_ClimbRunWindow(const vec3_t origin, const vec3_t start, const vec3_t aimdir, float speed)
{
	vec3_t d, a;
	float along, side, early, fly = LibVarValue("ig_hookfly", "1600");

	VectorSubtract(origin, start, d); d[2] = 0;
	if (VectorLength(d) <= IG_RUN_FIRE) return 1;
	VectorCopy(aimdir, a); a[2] = 0;
	if (VectorNormalize(a) < 1) return 0;
	along = DotProduct(d, a);
	side = fabs(d[0] * a[1] - d[1] * a[0]);
	/* Billy: with a slow hook (LMCTF) he prefires, the hook's flight in
	 * hand: the window opens as far before the start as the bot runs while
	 * the hook flies to the anchor (at most IG_RUN_EARLY) */
	early = fly > 100 ? speed * VectorLength(aimdir) / fly : 0;
	if (early < IG_RUN_FIRE) early = IG_RUN_FIRE;
	if (early > IG_RUN_EARLY) early = IG_RUN_EARLY;
	return along >= -early && along <= IG_RUN_PAST && side <= IG_RUN_SIDE;
}

int IG_ClimbFireNow(const vec3_t origin, const vec3_t viewoffset, const vec3_t viewangles, const vec3_t start, const vec3_t aimdir, int passent, float speed)
{
	vec3_t d, ideal, eye, end;
	aas_trace_t trace;
	float len, tol;

	if (!(IG_ClimbRun() & 1)) return 0;
	if (!IG_ClimbRunWindow(origin, start, aimdir, speed)) return 0;
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

static int IG_DrillAny(void);
static int IG_DrillAny(void);
static void IG_TakeOut(int reachnum, const char *why)
{
	aas_reachability_t *r;

	if (!ig_blocked || reachnum <= 0 || reachnum >= ig_blockedsize) return;
	if (IG_DrillAny()) return;   /* IG-37: a drill measures every try; nothing is taken out */
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

static int ig_drilling[MAX_CLIENTS];
static int IG_DrillAny(void)
{
	int i;

	for (i = 0; i < MAX_CLIENTS; i++)
		if (ig_drilling[i]) return 1;
	return 0;
}
int IG_CanFireHook(int client)
{
	if (client < 0 || client >= MAX_CLIENTS) return 0;
	if (ig_drilling[client]) return 0;   /* IG-37: the drill's climb is the only hook */
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
	for (i = 0; i < ig_hgcount; i++)
		botimport.Print(PRT_MESSAGE, "ig: high ground point %d at %.0f %.0f %.0f area %d\n", i,
						ig_hgpoint[i][0], ig_hgpoint[i][1], ig_hgpoint[i][2], ig_hgarea[i]);
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
static void IG_ClimbAimReset(void);
static void IG_ClimbTableAtLoad(void);
void IG_AdjustGrappleCosts(void)
{
	int i, n = 0, nodrop = 0, climbonly;
	float start, t;
	vec3_t d;
	aas_reachability_t *r;

	ig_hgcount = -1;    /* a new map: its high ground is worked out on first use */
	IG_ClimbAimReset(); /* IG-36: per-route climb aims are the old map's */
	IG_ClimbTableAtLoad(); /* IG-36: the map's climb table, before the routing */
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

/* The landing of the jump pad a bot is rising off (IG-40 pad flights in a
 * fight, ai_ig_plan.c): the nearest TRAVEL_JUMPPAD route out of the jump-pad
 * area at or up to 160 units below it. 1 and end, or 0 */
int IG_PadLaunchEnd(const vec3_t origin, vec3_t end)
{
	int dz, a, i, best = 0;
	float bd = 1e9f;

	for (dz = 0; dz <= 160 && !best; dz += 16) {
		vec3_t p;
		aas_areasettings_t *s;
		VectorCopy(origin, p); p[2] -= dz;
		a = AAS_PointAreaNum(p);
		if (!a || !(aasworld.areasettings[a].contents & AREACONTENTS_JUMPPAD)) continue;
		s = &aasworld.areasettings[a];
		for (i = 0; i < s->numreachableareas; i++) {
			aas_reachability_t *r = &aasworld.reachability[s->firstreachablearea + i];
			float d;
			if ((r->traveltype & TRAVELTYPE_MASK) != TRAVEL_JUMPPAD) continue;
			d = (r->start[0] - origin[0]) * (r->start[0] - origin[0]) + (r->start[1] - origin[1]) * (r->start[1] - origin[1]);
			if (d < bd) { bd = d; best = s->firstreachablearea + i; }
		}
	}
	if (!best) return 0;
	VectorCopy(aasworld.reachability[best].end, end);
	return 1;
}

void IG_NotePadFlight(int client, const vec3_t end)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	VectorCopy(end, ig_padend[client]);
	ig_padseen[client] = AAS_Time();
}

/* The pad a bot is walking onto (BotTravel_JumpPad), for the view before the launch */
static vec3_t ig_appstart[MAX_CLIENTS], ig_append[MAX_CLIENTS];
static float  ig_appseen[MAX_CLIENTS];

void IG_NotePadApproach(int client, const vec3_t start, const vec3_t end)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	VectorCopy(start, ig_appstart[client]);
	VectorCopy(end, ig_append[client]);
	ig_appseen[client] = AAS_Time();
}

int IG_PadApproach2(int client, vec3_t start, vec3_t end)
{
	float t;

	if (client < 0 || client >= MAX_CLIENTS) return 0;
	t = AAS_Time() - ig_appseen[client];
	if (ig_appseen[client] <= 0 || t < 0 || t > 0.25f) return 0;
	VectorCopy(ig_appstart[client], start);
	VectorCopy(ig_append[client], end);
	return 1;
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

/* IG-36: climb aim (libvar ig_climbaim, default 0). bspc's grapple route
 * hooks the point it proved -- often the corner of a small floor piece beside
 * an opening (q2rdm11's lava-room door: bots hooked the side of the corridor
 * and landed on its left edge). Billy hooks the frame around the opening and
 * air-strafes to its middle; sometimes he hooks the floor where he wants to
 * land, faster and less predictable. Per climb route, once:
 *   the SPOT: the point of the landing floor with the most room around it
 *     (floor in every direction, to IG_CA_ROOM), near the route's landing;
 *   a FRAME anchor: a wall above the spot (the frame or header of the
 *     opening, or the wall behind the ledge), in line with the spot and at
 *     a height the pull lifts the body over the lip from;
 *   a FLOOR anchor: the floor at the spot, when the bot can see it from the
 *     route's start.
 * Each climb takes one of the two (the floor IG_CA_FLOORCHANCE of the time),
 * every check of the route (aim, fire, progress, let-go) uses it, it lets go
 * once over the spot's floor near it, and after letting go it steers to the
 * spot until it lands (IG_ClimbSteer). Without either anchor, bspc's. */
#define IG_CA_GRID        32
#define IG_CA_SEARCH      160    /* the spot within this of the landing area's centre */
#define IG_CA_ROOM        96     /* floor this far around is room enough */
/* Floor anchors are worked out and logged, never used on a climb: drilled
 * (IG-37), a climb from below that hooks its landing floor landed 0 of 712
 * times -- the pull runs into the lip. Kept for drops and crossings (later). */
#define IG_CA_FLOORCHANCE 0.0f
#define IG_CA_LETGO       160    /* let go over the landing floor within this of the spot */
#define IG_CA_STEER       1.2f   /* seconds after letting go it steers to the spot */
#define IG_CA_EYE         22     /* Q2's viewheight */
#define IG_CA_SIDE        160    /* frame anchors this far to either side of the spot's line */

typedef struct
{
	unsigned char done, hasframe, hasfloor;   /* hasframe 2: over the spot (ceiling or sky) */
	unsigned char usebspc;       /* the map's climb table says bspc's anchor lands better here */
	float oz;                    /* the landing floor, in origin space */
	vec3_t spot, frame, floor;
} ig_climbaim_t;

static ig_climbaim_t *ig_climb;
static int ig_climbsize;
static int ig_capick[MAX_CLIENTS];       /* reachnum + 1 the pick is for */
static int ig_cafloor[MAX_CLIENTS];      /* 1: the floor anchor */
static float ig_catime[MAX_CLIENTS];
static vec3_t ig_caspot[MAX_CLIENTS];
static float ig_caoz[MAX_CLIENTS];
static float ig_casteer[MAX_CLIENTS];    /* steer to the spot until */

/* 0 off, 1 the frame anchor, 2 bspc's anchor with the landing (look ahead, let
 * go over the landing, air strafe / grapple down to its middle) */
int IG_ClimbAimOn(void)
{
	return (int) LibVarValue("ig_climbaim", "0");
}

/* IG-37: the drill decides per try; -1 = the libvar */
static int ig_caforce[MAX_CLIENTS] = { [0 ... MAX_CLIENTS - 1] = -1 };
void IG_ClimbAimForce(int client, int on)
{
	if (client >= 0 && client < MAX_CLIENTS) ig_caforce[client] = on;
}
static int ig_caover[MAX_CLIENTS];   /* IG-36: this climb hooks over the landing (ceiling or sky) */
/* IG-37: a new try: nothing of the last climb's landing carries over */
static int ig_clphase[MAX_CLIENTS];
void IG_ClimbAimClear(int client)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	ig_capick[client] = 0;
	ig_casteer[client] = 0;
	ig_clphase[client] = 0;
	ig_caover[client] = 0;
}

void IG_DrillSetActive(int client, int on)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	ig_drilling[client] = on;
	if (!on) ig_caforce[client] = -1;
}
static int IG_ClimbAimFor(int client)
{
	if (client >= 0 && client < MAX_CLIENTS && ig_caforce[client] >= 0) return ig_caforce[client];
	return IG_ClimbAimOn();
}

static void IG_ClimbAimReset(void)
{
	free(ig_climb);
	ig_climb = NULL;
	ig_climbsize = 0;
	memset(ig_capick, 0, sizeof(ig_capick));
	memset(ig_casteer, 0, sizeof(ig_casteer));
}

/* standable floor at origin height oz (within a step), no liquid */
static int IG_FloorAt(const vec3_t p, float oz)
{
	int area = AAS_PointAreaNum((vec_t *) p);
	aas_areasettings_t *st;

	if (area <= 0) return 0;
	st = &aasworld.areasettings[area];
	if (!(st->areaflags & AREA_GROUNDED)) return 0;
	if (st->contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_WATER)) return 0;
	return fabs(aasworld.areas[area].mins[2] - oz) <= 18;
}

/* how far the floor goes on in the worst of 8 directions, to IG_CA_ROOM */
static float IG_FloorRoom(const vec3_t p, float oz)
{
	int k;
	float d, room = IG_CA_ROOM;
	vec3_t q;

	for (k = 0; k < 8; k++)
	{
		float a = k * M_PI / 4;
		for (d = 16; d <= room; d += 16)
		{
			q[0] = p[0] + cos(a) * d;
			q[1] = p[1] + sin(a) * d;
			q[2] = p[2];
			if (!IG_FloorAt(q, oz)) { room = d - 16; break; }
		}
	}
	return room;
}

/* the body rides the pull from the start to IG_RUN_CLEAR of the way: clear? */
static int IG_PullClear(const vec3_t start, const vec3_t eye, const vec3_t anchor)
{
	vec3_t end, d;
	aas_trace_t trace;

	VectorSubtract(anchor, eye, d);
	VectorMA(start, IG_RUN_CLEAR, d, end);
	trace = AAS_TraceClientBBox((vec_t *) start, end, PRESENCE_NORMAL, -1);
	return trace.fraction >= 1;
}

/* Does the pull carry the body over the landing? OSP pulls the eye to the
 * anchor, so the body rides the line from the start to the anchor less a
 * viewheight. Traced as the player's box: somewhere before it hits anything
 * the feet must be above the landing floor with that floor under them --
 * the point the climb lets go (IG_ClimbAimLetGo). Drilled (IG-37): the
 * looser "clear for 80% of the way" passed anchors too low, whose pull ran
 * the body into the lip 30 units short of the top. */
static int IG_PullLands(const vec3_t start, const vec3_t anchor, float oz)
{
	vec3_t end, p, d;
	aas_trace_t trace;
	int k;

	vec3_t mins, maxs;
	bsp_trace_t btr;

	VectorCopy(anchor, end);
	end[2] -= IG_CA_EYE;
	/* a margin: the player's box 4 units wider, so the body clears a
	 * corner sticking out on the way up rather than grazing it */
	AAS_PresenceTypeBoundingBox(PRESENCE_NORMAL, mins, maxs);
	mins[0] -= 4; mins[1] -= 4; maxs[0] += 4; maxs[1] += 4;
	btr = AAS_Trace((vec_t *) start, mins, maxs, end, ENTITYNUM_NONE, CONTENTS_SOLID | CONTENTS_PLAYERCLIP);
	if (btr.startsolid) return 0;
	(void) trace;
	VectorSubtract(btr.endpos, start, d);
	for (k = 1; k <= 16; k++)
	{
		VectorMA(start, k / 16.0f, d, p);
		if (p[2] < oz + 8) continue;
		p[2] = oz + 2;
		if (IG_FloorAt(p, oz)) return 1;
	}
	return 0;
}

/* A hook point Billy would take: on the FRONT face of what it hooks (the face
 * looking back at the bot -- at q2rdm11's lava-room door the frame sticks out
 * on both sides and bspc hooks its side face), and on a flat part of it: the
 * same plane IG_FLAT_R units around, so not an edge or a corner. */
#define IG_FACE_DOT 0.7f
#define IG_FLAT_R   16
static int IG_FrontFlat(const vec3_t eye, const bsp_trace_t *hit, const vec3_t fdir)
{
	vec3_t t1, t2, up = { 0, 0, 1 }, p, d, end;
	bsp_trace_t tr;
	int k;
	float sx, sy;

	/* faces the bot: its normal points back at the eye, horizontally (a side
	 * face of a frame that sticks out looks across the climb, not at it).
	 * fdir NULL: no facing test (a ceiling over the landing) */
	if (fdir)
	{
		VectorSet(d, eye[0] - hit->endpos[0], eye[1] - hit->endpos[1], 0);
		VectorNormalize(d);
		VectorSet(p, hit->plane.normal[0], hit->plane.normal[1], 0);
		if (DotProduct(p, d) < IG_FACE_DOT) return 0;
	}
	/* flat: four points around it on its plane hit the same plane */
	CrossProduct(hit->plane.normal, up, t1);
	if (VectorNormalize(t1) < 0.1f) VectorSet(t1, 1, 0, 0);
	CrossProduct(hit->plane.normal, t1, t2);
	VectorNormalize(t2);
	for (k = 0; k < 4; k++)
	{
		sx = (k & 1) ? IG_FLAT_R : -IG_FLAT_R;
		sy = (k & 2) ? IG_FLAT_R : -IG_FLAT_R;
		VectorMA(hit->endpos, sx, t1, p);
		VectorMA(p, sy, t2, p);
		VectorSubtract(p, eye, d);
		VectorMA(eye, 1.2f, d, end);
		tr = AAS_Trace((vec_t *) eye, NULL, NULL, end, ENTITYNUM_NONE, CONTENTS_SOLID);
		if (tr.startsolid || tr.fraction >= 1) return 0;
		if (DotProduct(tr.plane.normal, hit->plane.normal) < 0.98f) return 0;
		VectorSubtract(tr.endpos, hit->endpos, d);
		if (fabs(DotProduct(d, hit->plane.normal)) > 4) return 0;
	}
	return 1;
}

static void IG_ClimbAimCompute(int reachnum, const aas_reachability_t *reach)
{
	ig_climbaim_t *ca = &ig_climb[reachnum];
	aas_area_t *area;
	vec3_t ref, p, eye, fdir, side, t, end, d, hit;
	float oz, best = -99999, score, room, x, y, f, h;
	static const float fs[] = { 0, 32, 64, 112, 176 };
	static const float hs[] = { 40, 80, 120, 170, 230, 290 };
	int i, j, l;
	bsp_trace_t tr;

	ca->done = 1;
	if (reach->areanum <= 0 || reach->areanum >= aasworld.numareas) return;
	area = &aasworld.areas[reach->areanum];
	oz = area->mins[2];
	ca->oz = oz;
	VectorSet(ref, area->center[0], area->center[1], oz + 2);
	VectorSubtract(ref, reach->start, d);
	d[2] = 0;
	f = VectorLength(d);
	/* the spot: most room, near the landing area, not far past it */
	for (x = -IG_CA_SEARCH; x <= IG_CA_SEARCH; x += IG_CA_GRID)
	{
		for (y = -IG_CA_SEARCH; y <= IG_CA_SEARCH; y += IG_CA_GRID)
		{
			VectorSet(p, ref[0] + x, ref[1] + y, oz + 2);
			if (!IG_FloorAt(p, oz)) continue;
			VectorSubtract(p, reach->start, d);
			d[2] = 0;
			if (VectorLength(d) > f + 128) continue;
			room = IG_FloorRoom(p, oz);
			if (room < 16) continue;
			score = room - 0.3f * sqrt(x * x + y * y);
			if (score > best) { best = score; VectorCopy(p, ca->spot); }
		}
	}
	if (best <= -99999) return;
	VectorCopy(reach->start, eye);
	eye[2] += IG_CA_EYE;
	VectorSubtract(ca->spot, reach->start, fdir);
	fdir[2] = 0;
	if (VectorNormalize(fdir) < 1) return;
	VectorSet(side, -fdir[1], fdir[0], 0);
	/* the frame: a wall above the spot, near its line, lifting the body over.
	 * Across as well as ahead: under an overhang the straight line hits its
	 * underside, and the frame is reached up past its sides (Billy's two grab
	 * points at the lava-room door, either side of the opening). */
	best = -99999;
	for (l = -IG_CA_SIDE; l <= IG_CA_SIDE; l += IG_CA_GRID)
	for (i = 0; i < (int) (sizeof(fs) / sizeof(fs[0])); i++)
	{
		for (j = 0; j < (int) (sizeof(hs) / sizeof(hs[0])); j++)
		{
			float beyond, lateral;
			int dbg = LibVarValue("ig_movedebug", "0") >= 3;

			VectorMA(ca->spot, fs[i], fdir, t);
			VectorMA(t, l, side, t);
			t[2] = oz - 24 + hs[j];
			VectorSubtract(t, eye, d);
			VectorMA(eye, 1.5f, d, end);
			tr = AAS_Trace(eye, NULL, NULL, end, ENTITYNUM_NONE, CONTENTS_SOLID);
			if (tr.startsolid || tr.fraction >= 1 || (tr.surface.flags & SURF_SKY))
			{
				if (dbg) botimport.Print(PRT_MESSAGE, "ig: ca %d f %.0f h %.0f nohit sky %d\n", reachnum, fs[i], hs[j], (tr.surface.flags & SURF_SKY) != 0);
				continue;
			}
			VectorCopy(tr.endpos, hit);
			if (dbg && (hit[2] < oz + 30 || hit[2] > oz + 260))
				botimport.Print(PRT_MESSAGE, "ig: ca %d f %.0f h %.0f low/high hit %.0f %.0f %.0f eye %.0f %.0f %.0f\n", reachnum, fs[i], hs[j], hit[0], hit[1], hit[2], eye[0], eye[1], eye[2]);
			if (hit[2] < oz + 30 || hit[2] > oz + 260) continue;
			VectorSubtract(hit, ca->spot, d);
			beyond = DotProduct(d, fdir);
			lateral = fabs(DotProduct(d, side));
			if (dbg) botimport.Print(PRT_MESSAGE, "ig: ca %d f %.0f h %.0f hit %.0f %.0f %.0f beyond %.0f lateral %.0f clear %d\n", reachnum, fs[i], hs[j],
									  hit[0], hit[1], hit[2], beyond, lateral, IG_PullClear(reach->start, eye, hit));
			if (beyond < -24 || beyond > 220 || lateral > IG_CA_SIDE) continue;
			{
				int ff = IG_FrontFlat(eye, &tr, fdir), pl = IG_PullLands(reach->start, hit, oz);
				if (dbg) botimport.Print(PRT_MESSAGE, "ig: cand %d hit %.0f %.0f %.0f n %.2f %.2f %.2f front/flat %d pull %d\n", reachnum,
										  hit[0], hit[1], hit[2], tr.plane.normal[0], tr.plane.normal[1], tr.plane.normal[2], ff, pl);
				if (!ff || !pl) continue;
			}
			h = hit[2] - (oz + 150);
			score = -0.5f * lateral - 0.3f * fabs(h) - 0.2f * (beyond > 48 ? beyond - 48 : 0);
			if (score > best) { best = score; VectorCopy(hit, ca->frame); ca->hasframe = 1; }
		}
	}
	/* No frame: an anchor OVER the spot -- a flat ceiling, or the sky where
	 * the server allows sky hooks (ig_hooksky) -- so the pull lifts the bot
	 * straight up onto the middle of the landing. Billy: bots climbing to
	 * q2rdm11's railgun overlook hooked the side wall and slid into the lava;
	 * they should hook "near/above where the railgun is placed". */
	if (!ca->hasframe)
	{
		static const float ov[][2] = { {0, 0}, {24, 0}, {-24, 0}, {0, 24}, {0, -24}, {48, 0}, {-48, 0}, {0, 48}, {0, -48} };
		int sky = LibVarValue("ig_hooksky", "0") != 0, k2;
		float hd;

		best = -99999;
		for (k2 = 0; k2 < (int) (sizeof(ov) / sizeof(ov[0])); k2++)
		{
			vec3_t base;
			bsp_trace_t up, see;

			/* what is straight above this point of the landing ... */
			VectorSet(base, ca->spot[0] + ov[k2][0], ca->spot[1] + ov[k2][1], oz + 8);
			VectorSet(t, base[0], base[1], oz + 1200);
			up = AAS_Trace(base, NULL, NULL, t, ENTITYNUM_NONE, CONTENTS_SOLID);
			if (up.startsolid || up.fraction >= 1) continue;
			VectorCopy(up.endpos, hit);
			hit[2] -= 1;
			/* ... and can the bot's hook reach it from the start */
			VectorSubtract(hit, eye, d);
			VectorMA(eye, 1.02f, d, t);
			see = AAS_Trace(eye, NULL, NULL, t, ENTITYNUM_NONE, CONTENTS_SOLID);
			VectorSubtract(see.endpos, hit, d);
			if (LibVarValue("ig_movedebug", "0") >= 3)
				botimport.Print(PRT_MESSAGE, "ig: over %d above %.0f %.0f %.0f sky %d seen-miss %.0f pull %d\n", reachnum,
								hit[0], hit[1], hit[2], (up.surface.flags & SURF_SKY) != 0, VectorLength(d), IG_PullLands(reach->start, hit, oz));
			if (see.startsolid || VectorLength(d) > 16) continue;
			if (hit[2] < oz + 60) continue;
			hd = sqrt(ov[k2][0] * ov[k2][0] + ov[k2][1] * ov[k2][1]);
			if (up.surface.flags & SURF_SKY)
			{
				if (!sky) continue;
			}
			else
			{   /* a ceiling: flat around the point */
				if (!IG_FrontFlat(eye, &up, NULL)) continue;
			}
			if (!IG_PullLands(reach->start, hit, oz)) continue;
			score = -hd;
			if (score > best) { best = score; VectorCopy(hit, ca->frame); ca->hasframe = 2; }
		}
	}
	/* the floor at the spot, if it can be seen from the start */
	VectorCopy(ca->spot, t);
	t[2] = oz - 24;
	VectorSubtract(t, eye, d);
	VectorMA(eye, 1.05f, d, end);
	tr = AAS_Trace(eye, NULL, NULL, end, ENTITYNUM_NONE, CONTENTS_SOLID);
	if (!tr.startsolid && tr.fraction < 1 && !(tr.surface.flags & SURF_SKY) &&
		fabs(tr.endpos[2] - (oz - 24)) <= 8)
	{
		VectorSubtract(tr.endpos, ca->spot, d);
		d[2] = 0;
		if (VectorLength(d) <= 40 && IG_PullClear(reach->start, eye, tr.endpos))
		{
			VectorCopy(tr.endpos, ca->floor);
			ca->hasfloor = 1;
		}
	}
	botimport.Print(PRT_MESSAGE, "ig: climbaim reach %d spot %.0f %.0f %.0f room %.0f frame %d %.0f %.0f %.0f floor %d bspc %.0f %.0f %.0f\n",
					reachnum, ca->spot[0], ca->spot[1], ca->spot[2], IG_FloorRoom(ca->spot, oz),
					ca->hasframe, ca->frame[0], ca->frame[1], ca->frame[2], ca->hasfloor,
					reach->end[0], reach->end[1], reach->end[2]);
}

/* The map's climb table (maps/<map>.climbs, next to its .aas, made from the
 * climb drill by the server repo's tools/climb-table.py): the routes where
 * bspc's anchor landed clearly more often than the frame anchor. Text:
 *   reachsize <n>      the .aas the drill ran on; another .aas ignores the table
 *   bspc <reachnum>    keeps bspc's anchor
 *   off <reachnum>     never landed in the drill, with either anchor: taken out
 *                      at map load (Billy: bots hooked the side wall below
 *                      q2rdm11's railgun overlook and slid into the lava twice
 *                      in a row -- route 3172, 0 of 18 drilled tries landed)
 * Without a table every route takes the frame anchor (where it has one). */
static void IG_ClimbTableLoad(void)
{
	char path[MAX_QPATH], *buf, *line, *next;
	fileHandle_t fp;
	int len, n = 0, off = 0, size = -1, r;

	Com_sprintf(path, sizeof(path), "maps/%s.climbs", aasworld.mapname);
	len = botimport.FS_FOpenFile(path, &fp, FS_READ);
	if (!fp || len <= 0) return;
	buf = calloc(len + 1, 1);
	if (!buf) { botimport.FS_FCloseFile(fp); return; }
	botimport.FS_Read(buf, len, fp);
	botimport.FS_FCloseFile(fp);
	for (line = buf; line && *line; line = next)
	{
		next = strchr(line, '\n');
		if (next) *next++ = 0;
		if (sscanf(line, "reachsize %d", &size) == 1) continue;
		if (sscanf(line, "bspc %d", &r) == 1 && size == aasworld.reachabilitysize && r > 0 && r < ig_climbsize)
		{
			ig_climb[r].usebspc = 1;
			n++;
		}
		if (sscanf(line, "off %d", &r) == 1 && size == aasworld.reachabilitysize && r > 0 && r < aasworld.reachabilitysize &&
			(aasworld.reachability[r].traveltype & TRAVELTYPE_MASK) == TRAVEL_GRAPPLEHOOK)
		{   /* before the routing is set up (IG_AdjustGrappleCosts): no cache to clear */
			aasworld.reachability[r].traveltype = (aasworld.reachability[r].traveltype & ~TRAVELTYPE_MASK) | TRAVEL_INVALID;
			off++;
		}
	}
	free(buf);
	if (size != aasworld.reachabilitysize)
		botimport.Print(PRT_MESSAGE, "ig: %s is for another navigation file (%d routes, this one %d), ignored\n", path, size, aasworld.reachabilitysize);
	else
		botimport.Print(PRT_MESSAGE, "ig: %s: %d climb route(s) keep bspc's anchor, %d never land: taken out\n", path, n, off);
}

static int IG_ClimbArray(void)
{
	if (ig_climb && ig_climbsize == aasworld.reachabilitysize) return 1;
	free(ig_climb);
	ig_climbsize = aasworld.reachabilitysize;
	ig_climb = calloc(ig_climbsize, sizeof(*ig_climb));
	if (!ig_climb) { ig_climbsize = 0; return 0; }
	if (LibVarValue("ig_climbtable", "1")) IG_ClimbTableLoad();
	return 1;
}

/* at map load, with the climb aim on, so "off" routes are out before routing */
static void IG_ClimbTableAtLoad(void)
{
	if (aasworld.reachabilitysize > 0 && LibVarValue("ig_climbaim", "0")) IG_ClimbArray();
}

static ig_climbaim_t *IG_ClimbAimGet(int reachnum, const aas_reachability_t *reach)
{
	if (reachnum <= 0 || reachnum >= aasworld.reachabilitysize) return NULL;
	if (!IG_ClimbArray()) return NULL;
	if (!ig_climb[reachnum].done) IG_ClimbAimCompute(reachnum, reach);
	return &ig_climb[reachnum];
}

/* IG-37: a route's landing spot and floor height, worked out if need be */
int IG_ClimbAimSpot(int reachnum, vec3_t spot, float *oz)
{
	ig_climbaim_t *ca;

	if (reachnum <= 0 || reachnum >= aasworld.reachabilitysize) return 0;
	ca = IG_ClimbAimGet(reachnum, &aasworld.reachability[reachnum]);
	if (!ca || (ca->spot[0] == 0 && ca->spot[1] == 0 && ca->spot[2] == 0)) return 0;
	VectorCopy(ca->spot, spot);
	*oz = ca->oz;
	return 1;
}

/* BotTravel_Grapple's route with the anchor this climb uses: 1 the frame
 * anchor (in out), 2 bspc's anchor but the landing is IG-36's, 0 neither */
int IG_ClimbAimReach(int client, int reachnum, const aas_reachability_t *reach, aas_reachability_t *out)
{
	ig_climbaim_t *ca;
	int mode;

	if (client < 0 || client >= MAX_CLIENTS || !(mode = IG_ClimbAimFor(client))) return 0;
	ca = IG_ClimbAimGet(reachnum, reach);
	if (!ca || (ca->spot[0] == 0 && ca->spot[1] == 0 && ca->spot[2] == 0)) return 0;
	/* the table's routes are plain bspc -- what the drill measured them against
	 * (the drill's forced aims ignore the table) */
	if (ca->usebspc && ig_caforce[client] < 0) return 0;
	if (mode == 2 || !ca->hasframe)
	{   /* bspc's anchor, IG-36's landing */
		ig_caover[client] = 0;
		ig_capick[client] = reachnum + 1;
		ig_cafloor[client] = -1;
		ig_catime[client] = AAS_Time();
		VectorCopy(ca->spot, ig_caspot[client]);
		ig_caoz[client] = ca->oz;
		ig_casteer[client] = AAS_Time() + IG_CA_STEER;
		return 2;
	}
	/* one pick per climb: the same while the route is in use (and was a frame pick) */
	if (ig_capick[client] != reachnum + 1 || ig_cafloor[client] < 0 || AAS_Time() - ig_catime[client] > 2 || AAS_Time() < ig_catime[client])
	{
		ig_capick[client] = reachnum + 1;
		ig_cafloor[client] = ca->hasframe ? (ca->hasfloor && random() < IG_CA_FLOORCHANCE) : 0;
	}
	ig_catime[client] = AAS_Time();
	*out = *reach;
	VectorCopy(ig_cafloor[client] ? ca->floor : ca->frame, out->end);
	ig_caover[client] = !ig_cafloor[client] && ca->hasframe == 2;
	VectorCopy(ca->spot, ig_caspot[client]);
	ig_caoz[client] = ca->oz;
	ig_casteer[client] = AAS_Time() + IG_CA_STEER;
	return 1;
}

/* Climbs in the planner (IG-40, libvar ig_planclimb; BOT-TRICKS T9): the
 * anchors a climb route can be hooked at, no side effects -- bspc's, and the
 * climb table's frame and landing floor (IG-36) when it has them. The planner
 * simulates a slingshot to each from where the bot is, moving. Returns the count. */
int IG_ClimbAnchorList(int reachnum, const aas_reachability_t *reach, vec3_t out[3])
{
	ig_climbaim_t *ca;
	int n = 0;

	VectorCopy(reach->end, out[n]); n++;
	if (!IG_ClimbAimFor(-1)) return n;
	ca = IG_ClimbAimGet(reachnum, reach);
	if (!ca || ca->usebspc) return n;
	if (ca->hasframe) { VectorCopy(ca->frame, out[n]); n++; }
	if (ca->hasfloor) { VectorCopy(ca->floor, out[n]); n++; }
	return n;
}

/* ...and while the planner owns a climb (it noted it this think or the last),
 * the stock grapple travel stands down unless its own hook is out */
static float ig_planclimbt[MAX_CLIENTS];
static int ig_planclimbr[MAX_CLIENTS];
void IG_PlanClimbNote(int client, int reachnum)
{
	if (client < 0 || client >= MAX_CLIENTS) return;
	ig_planclimbt[client] = AAS_Time(); ig_planclimbr[client] = reachnum;
}
int IG_PlanClimbOwns(int client, int reachnum)
{
	if (client < 0 || client >= MAX_CLIENTS) return 0;
	return ig_planclimbr[client] == reachnum && AAS_Time() - ig_planclimbt[client] < 0.25f && AAS_Time() >= ig_planclimbt[client];
}

/* IG-37: what this climb hooks: -1 bspc's anchor, 0 the frame, 1 the floor */
int IG_ClimbAimPick(int client, int reachnum)
{
	if (client < 0 || client >= MAX_CLIENTS || !IG_ClimbAimFor(client)) return -2;
	if (ig_capick[client] != reachnum + 1) return -2;
	return ig_cafloor[client];   /* -1 bspc's anchor with the landing, 0 frame, 1 floor */
}

/* the landing floor straight under the body: the player's box dropped down
 * stops at the floor's height. A map trace, not an AAS one: wedged against
 * the frame the bot's origin is in no area and the AAS trace starts solid. */
static int IG_FloorUnder(const vec3_t origin, float oz)
{
	vec3_t mins, maxs, end;
	bsp_trace_t tr;

	AAS_PresenceTypeBoundingBox(PRESENCE_NORMAL, mins, maxs);
	VectorSet(end, origin[0], origin[1], oz - 4);
	tr = AAS_Trace((vec_t *) origin, mins, maxs, end, ENTITYNUM_NONE, CONTENTS_SOLID | CONTENTS_PLAYERCLIP);
	return !tr.startsolid && tr.fraction < 1 && tr.endpos[2] >= oz - 6;
}

/* IG-36: would the bot, let go now, fly onto the landing? Its arc with
 * gravity and no steering, in IG_ARC_DT steps to IG_ARC_TIME, the player's
 * box traced along it: it lands if, coming down, the feet are over the
 * landing floor's height with that floor straight under them, before the
 * box hits anything. The earliest moment it holds is the time to let go --
 * held on to the threshold, the bot left rising at 740 u/s, flew 230 units
 * over the floor and drifted off the side (Billy: lava-room door). */
#define IG_ARC_DT    0.05f
#define IG_ARC_TIME  1.2f
#define IG_GRAVITY   800
static int IG_ArcLands(const vec3_t origin, const vec3_t velocity, float oz)
{
	vec3_t p, q, end;
	aas_trace_t tr;
	float t, vz;

	VectorCopy(origin, p);
	for (t = IG_ARC_DT; t <= IG_ARC_TIME; t += IG_ARC_DT)
	{
		q[0] = origin[0] + velocity[0] * t;
		q[1] = origin[1] + velocity[1] * t;
		q[2] = origin[2] + velocity[2] * t - 0.5f * IG_GRAVITY * t * t;
		tr = AAS_TraceClientBBox(p, q, PRESENCE_NORMAL, -1);
		if (tr.startsolid || tr.fraction < 1) return 0;
		vz = velocity[2] - IG_GRAVITY * t;
		if (vz <= 0 && q[2] >= oz + 2 && IG_FloorUnder(q, oz)) return 1;
		if (q[2] < oz - 64) return 0;
		VectorCopy(q, p);
	}
	return 0;
}

int IG_ClimbAimLetGoArc(int client, const vec3_t origin, const vec3_t velocity)
{
	if (client < 0 || client >= MAX_CLIENTS || !ig_capick[client]) return 0;
	if (!((int) LibVarValue("ig_climbland", "0") & 4)) return 0;
	return IG_ArcLands(origin, velocity, ig_caoz[client]);
}

/* ig_climbland bit 8: hold on until the pull stalls against the frame (the
 * wall takes the momentum), then let go at once over the landing. Without
 * it the bot let go at the threshold with the pull's speed and flew off the
 * side; holding on to the stuck rule (0.6 s) was "too long" (Billy). */
#define IG_STALL_LETGO 150
int IG_ClimbAimOverhead(int client)
{
	return client >= 0 && client < MAX_CLIENTS && ig_capick[client] && ig_caover[client];
}

/* an overhead anchor always holds to the stall: the pull ends under it, over
 * the middle of the landing, and the bot drops onto it (let go with the pull's
 * speed, it flew off the far side of the railgun overlook) */
int IG_ClimbAimHoldStall(int client)
{
	if (client < 0 || client >= MAX_CLIENTS || !ig_capick[client]) return 0;
	return ig_caover[client] || ((int) LibVarValue("ig_climbland", "0") & 8) != 0;
}

/* let go: the feet over the landing floor's height, with that floor under the
 * bot (not short of the lip), and near the spot; with bit 8, only once the
 * pull has stalled (speed under IG_STALL_LETGO) */
int IG_ClimbAimLetGo(int client, const vec3_t origin, const vec3_t velocity)
{
	vec3_t d, p;

	aas_trace_t trace;

	if (client < 0 || client >= MAX_CLIENTS || !ig_capick[client]) return 0;
	if (origin[2] < ig_caoz[client] + 8) return 0;
	if (IG_ClimbAimHoldStall(client) && VectorLength(velocity) > IG_STALL_LETGO) return 0;
	/* the landing floor under the body. Billy: the bot at the lava-room door
	 * held on too long -- the area test missed the doorway's floor and the
	 * bot hung at the frame until the stuck rule let go (0.6 s) */
	if (!IG_FloorUnder(origin, ig_caoz[client])) return 0;
	VectorSubtract(ig_caspot[client], origin, d);
	d[2] = 0;
	return VectorLength(d) <= IG_CA_LETGO;
}

/* IG-36: the way to look while climbing and landing: at the spot, level */
int IG_ClimbLookDir(int client, const vec3_t origin, vec3_t dir)
{
	if (client < 0 || client >= MAX_CLIENTS || !ig_capick[client]) return 0;
	VectorSubtract(ig_caspot[client], origin, dir);
	dir[2] = 0;
	return VectorNormalize(dir) > 24;
}

/* after letting go, in the air: the direction to the spot (air strafing to
 * the middle of the landing, as Billy does); 0 when there is nothing to do */
int IG_ClimbSteer(int client, const vec3_t origin, int grounded, int hooked, vec3_t dir)
{
	if (client < 0 || client >= MAX_CLIENTS || !IG_ClimbAimFor(client)) return 0;
	if (grounded) { ig_casteer[client] = 0; return 0; }
	if (hooked || ig_casteer[client] <= AAS_Time() || ig_casteer[client] > AAS_Time() + IG_CA_STEER + 1) return 0;
	VectorSubtract(ig_caspot[client], origin, dir);
	dir[2] = 0;
	if (VectorNormalize(dir) < 12) return 0;
	return 1;
}

/* IG-36 landing (libvar ig_climbland, bits; default 0 = IG_ClimbSteer's plain
 * push to the spot). After a climb lets go the bot still flies: at the
 * lava-room door it let go rising at 750 u/s, went 200 units over the floor
 * and drifted past the middle. Billy: "air strafe to stay in the middle of
 * the platform, or grapple down once over the landing zone".
 *   1 air strafe: wish the velocity that takes it to the spot (brake the
 *     sideways speed it arrived with, then drift in), looking where it goes;
 *   2 grapple down: over the landing floor and still IG_CL_HIGH above it,
 *     look at the floor at the spot and hook it; the pull plants it there
 *     and it lets go IG_CL_LOW above the floor (or after IG_CL_PULL s).
 * IG_ClimbLand says what to do this think: a move direction, a view, and
 * whether to fire or let go of the hook. */
#define IG_CL_HIGH  48     /* hook down only this far above the landing floor */
#define IG_CL_LOW   20     /* let go this close above it */
#define IG_CL_PULL  0.6f
#define IG_CL_AIM   5      /* degrees off the floor point it may fire */
#define IG_CL_TIME  0.6f   /* seconds to reach the spot the air strafe plans for */

/* ig_clphase (declared with IG_ClimbAimClear): 0 flying, 1 hooked down, 2 done */
static float ig_clfire[MAX_CLIENTS];
static vec3_t ig_clanchor[MAX_CLIENTS];

int IG_ClimbLand(int client, const vec3_t origin, const vec3_t velocity, const vec3_t viewangles, const vec3_t viewoffset,
				 int grounded, int hooked, vec3_t movedir, vec3_t view, int *fire, int *release)
{
	int mode = (int) LibVarValue("ig_climbland", "0");
	vec3_t d, eye, end, ang, want;
	aas_trace_t down;
	bsp_trace_t tr;
	float oz;

	*fire = *release = 0;
	VectorClear(movedir);
	if (client < 0 || client >= MAX_CLIENTS || !IG_ClimbAimFor(client) || !ig_capick[client]) return 0;
	if (grounded)
	{
		if (ig_clphase[client] == 1) *release = 1;
		ig_casteer[client] = 0;
		ig_clphase[client] = 0;
		return *release;
	}
	if (ig_casteer[client] <= AAS_Time() || ig_casteer[client] > AAS_Time() + IG_CA_STEER + 1)
	{
		ig_clphase[client] = 0;
		return 0;
	}
	if (hooked && ig_clphase[client] != 1) return 0;      /* still the climb itself */
	oz = ig_caoz[client];
	if (!mode)
	{   /* the plain push of the first version */
		if (!IG_ClimbSteer(client, origin, 0, 0, movedir)) return 0;
		IG_ClimbLookDir(client, origin, d);
		vectoangles(d, view);
		return 1;
	}
	VectorAdd(origin, viewoffset, eye);
	if (ig_clphase[client] == 1)
	{   /* hooked down: keep it, let go near the floor */
		IG_GrappleTravelNote(client);         /* not an orphan for the watchdog */
		VectorSubtract(ig_clanchor[client], eye, d);
		vectoangles(d, view);
		if (origin[2] <= oz + IG_CL_LOW || AAS_Time() - ig_clfire[client] > IG_CL_PULL)
		{
			*release = 1;
			ig_clphase[client] = 2;
		}
		return 1;
	}
	/* grapple down: over the landing floor, high, hook free */
	/* the hook wait itself, not IG_CanFireHook: the drill blocks other hooks */
	if ((mode & 2) && ig_clphase[client] == 0 && origin[2] > oz + IG_CL_HIGH && AAS_Time() - ig_lastfire[client] >= IG_HOOK_WAIT)
	{
		if (IG_FloorUnder(origin, oz))
		{
			VectorSet(end, ig_caspot[client][0], ig_caspot[client][1], oz - 24 - 8);
			tr = AAS_Trace(eye, NULL, NULL, end, ENTITYNUM_NONE, CONTENTS_SOLID);
			VectorSubtract(tr.endpos, end, d);
			if (!tr.startsolid && tr.fraction < 1 && VectorLength(d) < 24)
			{
				VectorSubtract(tr.endpos, eye, d);
				vectoangles(d, view);
				if (fabs(AngleMod(view[0] - viewangles[0] + 180) - 180) < IG_CL_AIM &&
					fabs(AngleMod(view[1] - viewangles[1] + 180) - 180) < IG_CL_AIM)
				{
					VectorCopy(tr.endpos, ig_clanchor[client]);
					*fire = 1;
					ig_clphase[client] = 1;
					ig_clfire[client] = AAS_Time();
					IG_NoteHookFire(client);
				}
				return 1;    /* turning to the floor point */
			}
		}
	}
	/* air strafe: the velocity that takes it to the spot in IG_CL_TIME */
	VectorSubtract(ig_caspot[client], origin, d);
	d[2] = 0;
	VectorScale(d, 1.0f / IG_CL_TIME, want);
	if (VectorLength(want) > 300) { VectorNormalize(want); VectorScale(want, 300, want); }
	VectorSet(d, want[0] - velocity[0], want[1] - velocity[1], 0);
	if (VectorLength(d) < 20) return 0;
	VectorNormalize(d);
	if (mode & 1) VectorCopy(d, movedir);
	/* look where it is going, level */
	VectorSet(ang, velocity[0], velocity[1], 0);
	if (VectorLength(ang) < 50) VectorCopy(want, ang);
	vectoangles(ang, view);
	view[0] = 0;
	return 1;
}

/* IG-43 pad routes (libvar ig_padreach, default 0; only with the hook,
 * ig_rs_startgrapple > 0). A Q2 trigger_push that throws straight up has no
 * landing of its own: bspc makes a pad route only where the throw, or a
 * little air control (under 150 u/s across), lands somewhere, so the bots
 * never take q2rdm11's two up-pads -- and one dropped onto them bounced
 * forever (IG-42). Billy takes them with his hook: straight up, a turn, at the
 * top a level hook along the upper walkway, 938 u/s (recorded 2026-11-25).
 * Here, at load, before the routing: for each such pad with no pad route out,
 * the top of its arc, and from it the floors in sight 150-900 units across,
 * up to 350 below the top (and 32 above), no lava or slime, in the same
 * routing cluster -- the farthest in each of 8 directions -- become
 * TRAVEL_JUMPPAD routes. The bot then takes the pad like any other; in the
 * air the planner's pad top (ai_ig_plan.c) hooks it there, or it falls back
 * onto the pad and IG-42's steering takes it off. The pad area's block of
 * routes moves to the end of the table, so no other route's number changes
 * (the climb tables and recorded routes name routes by number). */
extern int AAS_GetJumpPadInfo(int ent, vec3_t areastart, vec3_t absmins, vec3_t absmaxs, vec3_t velocity);

#define IG_PR_DIRS 8
static int ig_padreaches[64], ig_numpadreaches;

int IG_PadReachList(int *out, int max)	/* IG-43: the routes added, for the pad drill */
{
	int i;

	for (i = 0; i < ig_numpadreaches && i < max; i++) out[i] = ig_padreaches[i];
	return i;
}

/* IG-6's per-route marks: room for route numbers added at load (IG-43, IG-46) */
void IG_BlockedResize(void)
{
	if (ig_blocked && ig_blockedsize < aasworld.reachabilitysize) {
		unsigned char *nb = calloc(aasworld.reachabilitysize, 1);

		if (nb) { memcpy(nb, ig_blocked, ig_blockedsize); free(ig_blocked); ig_blocked = nb; ig_blockedsize = aasworld.reachabilitysize; }
	}
}

void IG_PadReachAtLoad(void)
{
	int ent, a, i, k, added = 0, pads = 0, skipped = 0, padareas[16], npad, newsize;
	char classname[MAX_EPAIRKEY];
	vec3_t start, mins, maxs, vel, apex, p, d;
	aas_link_t *areas, *link;
	aas_reachability_t extra[16][IG_PR_DIRS], *nr;
	int nextra[16];
	float g = aassettings.phys_gravity > 0 ? aassettings.phys_gravity : 800;

	ig_numpadreaches = 0;
	if (!LibVarValue("ig_padreach", "0") || LibVarGetValue("ig_rs_startgrapple") <= 0) return;
	for (ent = AAS_NextBSPEntity(0); ent; ent = AAS_NextBSPEntity(ent)) {
		int best[IG_PR_DIRS];
		float bestd[IG_PR_DIRS], h;

		if (!AAS_ValueForBSPEpairKey(ent, "classname", classname, MAX_EPAIRKEY) || strcmp(classname, "trigger_push")) continue;
		if (!AAS_GetJumpPadInfo(ent, start, mins, maxs, vel)) continue;
		if (LibVarValue("ig_padreachdebug", "0")) botimport.Print(PRT_MESSAGE, "ig: push ent %d at %.0f %.0f %.0f vel %.0f %.0f %.0f\n", ent, start[0], start[1], start[2], vel[0], vel[1], vel[2]);
		if (vel[0] * vel[0] + vel[1] * vel[1] > 200 * 200 || vel[2] < 300) continue;	/* (nearly) straight up only: q2rdm11's throw 89-113 across */
		areas = AAS_LinkEntityClientBBox(mins, maxs, -1, PRESENCE_CROUCH);
		if (LibVarValue("ig_padreachdebug", "0"))
			for (link = areas; link; link = link->next_area) {
				int r, n = aasworld.areasettings[link->areanum].numreachableareas, f = aasworld.areasettings[link->areanum].firstreachablearea;
				botimport.Print(PRT_MESSAGE, "ig: padpush at %.0f %.0f %.0f vel %.0f %.0f %.0f area %d jumppad %d reaches %d\n", start[0], start[1], start[2], vel[0], vel[1], vel[2],
								link->areanum, AAS_AreaJumpPad(link->areanum), n);
				for (r = 0; r < n; r++)
					botimport.Print(PRT_MESSAGE, "ig:   reach %d type %d to %d\n", f + r, aasworld.reachability[f + r].traveltype & TRAVELTYPE_MASK, aasworld.reachability[f + r].areanum);
			}
		npad = 0;
		for (link = areas; link; link = link->next_area) {
			int r, has = 0, n = aasworld.areasettings[link->areanum].numreachableareas, f = aasworld.areasettings[link->areanum].firstreachablearea;

			if (!AAS_AreaJumpPad(link->areanum)) continue;
			for (r = 0; r < n; r++)
				if ((aasworld.reachability[f + r].traveltype & TRAVELTYPE_MASK) == TRAVEL_JUMPPAD) has = 1;
			if (!has && npad < 16) padareas[npad++] = link->areanum;
		}
		AAS_UnlinkFromAreas(areas);
		if (!npad) continue;
		pads++;
		h = vel[2] * vel[2] / (2 * g);
		VectorCopy(start, apex);
		apex[2] += h - 8;
		for (k = 0; k < IG_PR_DIRS; k++) { best[k] = 0; bestd[k] = 0; }
		for (a = 1; a < aasworld.numareas; a++) {
			aas_area_t *ar = &aasworld.areas[a];
			aas_trace_t tr;
			float hd;

			if (!AAS_AreaGrounded(a)) continue;
			if (aasworld.areasettings[a].contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_JUMPPAD)) continue;
			VectorSet(p, ar->center[0], ar->center[1], ar->mins[2] + 1);
			if (AAS_PointAreaNum(p) != a) continue;
			if (p[2] < apex[2] - 350 || p[2] > apex[2] + 32) continue;
			VectorSubtract(p, apex, d);
			d[2] = 0;
			hd = VectorLength(d);
			if (hd < 150 || hd > 900) continue;
			for (i = 0; i < npad; i++) {	/* the routing ignores a route out of its cluster */
				int c1 = aasworld.areasettings[padareas[i]].cluster, c2 = aasworld.areasettings[a].cluster;
				if (c1 == c2 || c1 < 0 || c2 < 0) break;
			}
			if (i == npad) { skipped++; continue; }
			k = ((int) floor((atan2(d[1], d[0]) + M_PI) / (2 * M_PI) * IG_PR_DIRS)) % IG_PR_DIRS;
			if (hd <= bestd[k]) continue;
			tr = AAS_TraceClientBBox(apex, p, PRESENCE_NORMAL, -1);
			if (tr.startsolid || tr.fraction < 1) continue;
			best[k] = a; bestd[k] = hd;
		}
		for (i = 0; i < npad; i++) {
			nextra[i] = 0;
			for (k = 0; k < IG_PR_DIRS; k++) {
				aas_reachability_t *r;
				int c1 = aasworld.areasettings[padareas[i]].cluster, c2;

				if (!best[k]) continue;
				c2 = aasworld.areasettings[best[k]].cluster;
				if (!(c1 == c2 || c1 < 0 || c2 < 0)) continue;
				r = &extra[i][nextra[i]++];
				Com_Memset(r, 0, sizeof(*r));
				r->areanum = best[k];
				r->facenum = (int) vel[2];		/* as bspc's pad routes: the throw's z speed */
				r->edgenum = 0;					/* and its speed across */
				VectorCopy(start, r->start);
				VectorSet(r->end, aasworld.areas[best[k]].center[0], aasworld.areas[best[k]].center[1], aasworld.areas[best[k]].mins[2] + 1);
				/* up, a hook's flight, the pull across, and a price for the trick */
				r->traveltime = (unsigned short) (100 * (vel[2] / g + 0.3f + bestd[k] / 800 + LibVarValue("ig_padreachprice", "0.2")));
				r->traveltype = TRAVEL_JUMPPAD;
			}
			if (!nextra[i]) continue;
			/* this pad area's block, moved to the end with the new routes after it */
			{
				aas_areasettings_t *as = &aasworld.areasettings[padareas[i]];

				newsize = aasworld.reachabilitysize + as->numreachableareas + nextra[i];
				nr = (aas_reachability_t *) GetClearedMemory(newsize * sizeof(aas_reachability_t));
				Com_Memcpy(nr, aasworld.reachability, aasworld.reachabilitysize * sizeof(aas_reachability_t));
				Com_Memcpy(nr + aasworld.reachabilitysize, aasworld.reachability + as->firstreachablearea, as->numreachableareas * sizeof(aas_reachability_t));
				Com_Memcpy(nr + aasworld.reachabilitysize + as->numreachableareas, extra[i], nextra[i] * sizeof(aas_reachability_t));
				as->firstreachablearea = aasworld.reachabilitysize;
				as->numreachableareas += nextra[i];
				FreeMemory(aasworld.reachability);
				aasworld.reachability = nr;
				aasworld.reachabilitysize = newsize;
			}
			for (k = 0; k < nextra[i]; k++)
				if (ig_numpadreaches < 64) ig_padreaches[ig_numpadreaches++] = aasworld.reachabilitysize - nextra[i] + k;
			for (k = 0; k < nextra[i]; k++)
				botimport.Print(PRT_MESSAGE, "ig: padreach area %d -> %d end %.0f %.0f %.0f time %d top %.0f\n", padareas[i], extra[i][k].areanum,
								extra[i][k].end[0], extra[i][k].end[1], extra[i][k].end[2], extra[i][k].traveltime, apex[2]);
			added += nextra[i];
		}
	}
	if (added) IG_BlockedResize();
	if (pads) botimport.Print(PRT_MESSAGE, "ig: %d pad routes added from %d straight-up pads (%d floors in another cluster)\n", added, pads, skipped);
}
