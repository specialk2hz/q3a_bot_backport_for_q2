/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_sling.c -- slingshot hooking, modelled on a person's (IG-9).
 *
 * A movement technique on top of the route, not a route. Tuned to Billy's
 * own hooking (server repo, bench/billy-session1): 88% of his hooks go out
 * in the air -- he jumps first, for the angle and the speed --, the anchor is
 * short (median 275 units) and as often below eye level as above (median 12
 * degrees down: he hooks the floor or a low wall ahead and the pull turns
 * the jump's height into forward speed), he is on the hook about 0.2 s and
 * lets go around 60% of the way, and 59% of his hooks follow another within
 * 1.5 s: a flow of short hooks, not one big pull.
 *
 * So, while a bot runs a long clear stretch of route on the ground:
 *   1. it jumps;
 *   2. near the top of the jump it picks an anchor along the route, 180-450
 *      units ahead, 18 degrees below to 12 above the eye, solid, not sky,
 *      with the body's path and the floor beyond checked, and aims;
 *   3. aimed, it hooks; latched, it rides the pull for a fraction of the
 *      distance left and lets go;
 *   4. still in the air with the route running on, it may hook again
 *      (OSP's half second between hooks, then aim and fire); back on the
 *      ground, it jumps again.
 *
 * Called from every node that follows a route: seek LTG and NBG, battle chase,
 * retreat and NBG (Billy hooks ~18 times a minute, fights included).
 *
 * libvars (src/ig/ig_bot.c): ig_sling 1 turns it on (default 0), with IG-6's
 * hook wiring (ig_hookdetect). Lines "ig: slingshot <bot> ..." are what
 * igbench counts.
 */

#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"
#include "chars.h"

/* botlib/be_ig_hook.c */
void IG_GrappleTravelNote(int client);
void IG_NoteHookFire(int client);
int  IG_CanFireHook(int client);
int  IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void EA_Jump(int client);
/* game_q3/ai_main.c. Undeclared, C took it for an int function and every
 * aim check here read garbage: hooks left up to 90 degrees off their anchor. */
float AngleDifference(float ang1, float ang2);

#define IG_LOOKAHEAD     700    /* how far along the route it looks */
#define IG_MIN_RUN       160    /* the route must run on this far, horizontally */
#define IG_ANCHOR_MIN    250    /* Billy pulls from about 275 out: closer is no slingshot */
#define IG_ANCHOR_MAX    650
#define IG_JUMP_AIM      0.15f  /* seconds into the jump to pick the anchor */
#define IG_AIM_TIME      0.7f   /* seconds to get aimed and fire */
#define IG_AIM_TOL       8      /* degrees, now that the check works (AngleDifference) */
#define IG_FLY_MAX       0.55f  /* seconds for the hook to latch: 650 units at 1650 u/s is 0.4 */
#define IG_RELEASE_FRAC  0.6f   /* let go this far along the pull ... */
#define IG_RELEASE_SPEED 900    /* ... or this fast, after a moment's pull */
#define IG_HOOK_MAX      0.15f  /* seconds pulled, from the latch: grapple and QUICKLY let go (Billy) */
#define IG_STALL_SPEED   150    /* pulled but this slow: the body is caught */
#define IG_LATCH_RADIUS  650    /* how far from the aimed anchor the hook may latch: a
                                  * floor hook at a shallow angle lands far along the
                                  * floor for a few degrees of aim; it is the bot's
                                  * hook if it is in front of it (IG_InFront) */
#define IG_REHOOK        0.5f   /* OSP's hook_wait: the soonest it can hook again */
#define IG_CHAIN_WINDOW  4.0f   /* in the air this long after a let-go, it may chain */
#define IG_CHAIN_CHANCE  0.95f  /* how often it does (Billy: 59% within 1.5 s, at 18 hooks a minute) */
#define IG_COOLDOWN      0.4f   /* after a sequence ends, before the next jump */

enum { IG_SLING_IDLE, IG_SLING_JUMPED, IG_SLING_AIMING, IG_SLING_HOOKED, IG_SLING_AFTER };

typedef struct {
	int    state;
	float  time;        /* when the state began */
	float  cooldown;    /* no new sequence before */
	vec3_t anchor;
	float  latched;     /* when the hook latched on, 0: still flying */
	vec3_t latchpos;    /* where the bot was then */
	float  remain;      /* eye to anchor then: the pull's length */
	int    chain;       /* hooks in this sequence */
	int    stairs;      /* IG-20: a stairwell hook, up to topz */
	float  topz;
} ig_sling_t;

static ig_sling_t ig_sling[MAX_CLIENTS];

/* Keep the momentum (Billy: jump before the grapple, and keep jumping to keep
 * the speed). Quake II applies ground friction only to a player on the
 * ground, so a jump on the landing frame keeps the speed a hook gave. For
 * IG_HOP_TIME after a let-go, a bot that lands still faster than
 * IG_HOP_SPEED, with safe floor ahead the way it is moving, jumps at once --
 * up to IG_HOP_MAX times. */
#define IG_HOP_TIME   1.5f
#define IG_HOP_SPEED  350
#define IG_HOP_MAX    2      /* Billy: about two jumps, then stop or sling again */
static float ig_hopuntil[MAX_CLIENTS];
static int   ig_hops[MAX_CLIENTS];

static void IG_HopStart(int c)
{
	if (c < 0 || c >= MAX_CLIENTS) return;
	ig_hopuntil[c] = FloatTime() + IG_HOP_TIME;
	ig_hops[c] = 0;
}

static int IG_SafeCorridor(bot_state_t *bs, const vec3_t flat, float length);
static int IG_OnGround(bot_state_t *bs);

/* 1 if it hopped this think */
static int IG_Hop(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t v;
	float speed;

	if (c < 0 || c >= MAX_CLIENTS) return 0;
	if (ig_hopuntil[c] > FloatTime() + IG_HOP_TIME + 1) ig_hopuntil[c] = 0;     /* a new map */
	if (FloatTime() > ig_hopuntil[c] || ig_hops[c] >= IG_HOP_MAX || !IG_OnGround(bs)) return 0;
	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	speed = VectorNormalize(v);
	if (speed < IG_HOP_SPEED || !IG_SafeCorridor(bs, v, 400)) return 0;
	EA_Jump(c);
	ig_hops[c]++;
	ig_hopuntil[c] = FloatTime() + IG_HOP_TIME;   /* each hop extends it */
	BotAI_Print(PRT_MESSAGE, "ig: hop %d speed %.0f\n", c, speed);
	return 1;
}

/* the combat hook's state (IG-10), here because the slingshot checks it */
#define IG_FIGHT_COOLDOWN 2.5f
#define IG_FIGHT_PULL     0.25f
#define IG_FIGHT_CHANCE   0.5f  /* per think, just after a shot */
#define IG_FIGHT_IDLE     0.03f /* per think otherwise */

enum { IG_FIGHT_NONE, IG_FIGHT_AIMING, IG_FIGHT_HOOKED };

typedef struct {
	int    state;
	float  time, cooldown, latched;
	vec3_t anchor, latchpos;
	float  remain;
	/* reposition (Billy): after a kill, sometimes; after two misses in a row */
	int    shooting;    /* firing animation last think: a shot is its rising edge */
	float  shot;        /* when the last shot went, 0: judged */
	float  lastkill;    /* killedenemy_time already counted */
	int    misses;      /* in a row */
	int    urge;        /* 1: hook away now */
} ig_fight_t;

#define IG_REPO_AFTER_KILL 0.4f    /* how often a kill is followed by a reposition */
#define IG_REPO_MISSES     2
#define IG_HIT_WINDOW      0.4f    /* a rail that kills, kills at once in instagib */

static ig_fight_t ig_fight[MAX_CLIENTS];

static int IG_SlingOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_sling") && trap_Cvar_VariableIntegerValue("ig_hookdetect");
}

static void IG_ViewAt(bot_state_t *bs, const vec3_t p)
{
	vec3_t dir;

	VectorSubtract(p, bs->eye, dir);
	vectoangles(dir, bs->ideal_viewangles);
	bs->flags |= BFL_IDEALVIEWSET;
}

/* A flick, the way a player snaps onto a hook spot: the view jumps to the
 * spot instead of easing in (Q3's view turn covers a fraction of what is left
 * each think, so a 20-30 degree tip took most of a jump: 5 000 slingshot aims
 * of 16 000 ran out of time in a test). */
static void IG_FlickAt(bot_state_t *bs, const vec3_t p)
{
	int i;

	IG_ViewAt(bs, p);
	/* the view turn keeps an angular speed between server frames
	 * (BotChangeViewAngles): left in, it carried the view on past the flick
	 * -- the game fired 14 degrees down asked, -2 to -7 got */
	for (i = 0; i < 2; i++) {
		bs->viewangles[i] = AngleMod(bs->ideal_viewangles[i]);
		bs->viewanglespeed[i] = 0;
	}
}

/* Flick onto p; true on the think AFTER a flick onto it, when the game's view
 * (the last usercmd's) is the flicked one and a hook can go. A think spent in
 * between with another view (a node change, the battle aim) starts it over:
 * hooks fired then flew where the bot had been looking. Every hook a bot
 * aims goes through this: aiming by the eased view turn took up to 0.7 s,
 * mid-fight looking away from the enemy -- rail accuracy 48% -> 36%. */
static float ig_flicked[MAX_CLIENTS];
static int IG_FlickReady(bot_state_t *bs, const vec3_t p)
{
	int c = bs->client;
	float since;
	int ready;

	if (c < 0 || c >= MAX_CLIENTS) return 0;
	since = FloatTime() - ig_flicked[c];
	ready = ig_flicked[c] > 0 && since > 0.001f && since < 0.15f;
	IG_FlickAt(bs, p);
	ig_flicked[c] = FloatTime();
	return ready;
}


static int IG_OnGround(bot_state_t *bs)
{
	return bs->cur_ps.groundEntityNum != ENTITYNUM_NONE;
}

/* In a jump pad's flight: the pad hook's, and no other hook's. Slingshot
 * chains and combat hooks fired in the arc hooked the floor below and short
 * of the landing (Billy, q2rdm11's big room). */
static int IG_InPadFlight(bot_state_t *bs)
{
	vec3_t land;

	return !IG_OnGround(bs) && IG_PadFlight(bs->client, land);
}

#define IG_LIQUIDS (CONTENTS_LAVA | CONTENTS_SLIME)

/* A trace to an anchor that also sees lava and slime: MASK_SOLID passes
 * through a pool's surface and finds the floor under it, a "solid" anchor
 * that pulls the bot in. False if it hit a liquid or the sky. */
static int IG_AnchorTraceSky(bot_state_t *bs, vec3_t end, bsp_trace_t *tr, int allowsky)
{
	BotAI_Trace(tr, bs->eye, NULL, NULL, end, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
	if (tr->fraction >= 1 || tr->startsolid) return 0;
	if (tr->contents & IG_LIQUIDS) return 0;
	if ((tr->surface.flags & SURF_SKY) && !allowsky) return 0;
	return 1;
}

/* The sky is an anchor when the server allows hooking it (hook_sky, pushed
 * as ig_hooksky; on in instagib): open areas have no other. */
static int IG_AnchorTrace(bot_state_t *bs, vec3_t end, bsp_trace_t *tr)
{
	return IG_AnchorTraceSky(bs, end, tr, trap_Cvar_VariableIntegerValue("ig_hooksky"));
}

/* Floor under p, within 700 units, and not lava or slime. */
static int IG_SafeGround(bot_state_t *bs, const vec3_t p)
{
	vec3_t down;
	bsp_trace_t tr;

	VectorCopy(p, down);
	down[2] -= 700;
	BotAI_Trace(&tr, (vec_t *)p, NULL, NULL, down, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
	return tr.fraction < 1 && !(tr.contents & IG_LIQUIDS);
}

/* Safe floor every IG_CORRIDOR_STEP along `flat` (horizontal, unit) from
 * the bot out to `length`: where a hooked bot flies. Released at ~975 u/s it
 * travels up to a second more, 800 units past the let-go -- checking only
 * the let-go and the anchor put the bots in q2rdm11's lava (105 deaths in
 * 10 matches traced back to slingshots and combat hooks). */
#define IG_CORRIDOR_STEP 96
static int IG_SafeCorridor(bot_state_t *bs, const vec3_t flat, float length)
{
	vec3_t p;
	float t;

	for (t = IG_CORRIDOR_STEP; t <= length; t += IG_CORRIDOR_STEP) {
		VectorMA(bs->origin, t, flat, p);
		if (!IG_SafeGround(bs, p)) return 0;
	}
	return 1;
}

/* IG-29 (libvar ig_walkway): the corridor also stays on the walkway. The
 * slingshot aims at the route a few hundred units on, in a straight line;
 * on a walkway that bends over lava (q2rdm11) that line cuts the corner off
 * the edge, over lower floor or lava. Here the floor along the line may not
 * fall more than IG_WALKWAY_DROP below the lower of the bot's feet and the
 * route point it aims at, so a route that goes down still may. */
#define IG_WALKWAY_DROP 48
#define IG_WALKWAY_SIDE 192     /* how far to each side lava counts as "beside" */
/* Lava or slime beside the line (either side, IG_WALKWAY_SIDE off), out to
 * where the flight ends: only there does leaving the walkway matter. Applied
 * everywhere, the rule cost q2dm1 (no lava) a third of its slingshots. */
static int IG_LavaBeside(bot_state_t *bs, const vec3_t flat, float length)
{
	vec3_t p, side, down;
	bsp_trace_t tr;
	float t;
	int k;

	VectorSet(side, -flat[1], flat[0], 0);
	for (t = IG_CORRIDOR_STEP; t <= length; t += IG_CORRIDOR_STEP) {
		for (k = -1; k <= 1; k += 2) {
			VectorMA(bs->origin, t, flat, p);
			VectorMA(p, k * IG_WALKWAY_SIDE, side, p);
			VectorCopy(p, down);
			down[2] -= 700;
			BotAI_Trace(&tr, p, NULL, NULL, down, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
			if (!tr.startsolid && tr.fraction < 1 && (tr.contents & IG_LIQUIDS)) return 1;
		}
	}
	return 0;
}

static int IG_WalkwayCorridor(bot_state_t *bs, const vec3_t flat, float length, const vec3_t target)
{
	vec3_t p, down;
	bsp_trace_t tr;
	float t, minz;

	if (!IG_SafeCorridor(bs, flat, length)) return 0;
	if (!trap_Cvar_VariableIntegerValue("ig_walkway")) return 1;
	if (!IG_LavaBeside(bs, flat, length + 300)) return 1;
	minz = bs->origin[2] - 24;                          /* the feet */
	if (target && target[2] - 24 < minz) minz = target[2] - 24;
	minz -= IG_WALKWAY_DROP;
	for (t = IG_CORRIDOR_STEP; t <= length; t += IG_CORRIDOR_STEP) {
		VectorMA(bs->origin, t, flat, p);
		VectorCopy(p, down);
		down[2] -= 700;
		BotAI_Trace(&tr, p, NULL, NULL, down, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
		if (tr.startsolid) continue;                    /* a step up: still on it */
		if (tr.endpos[2] < minz) return 0;
	}
	return 1;
}

int IG_LavaEscape(bot_state_t *bs);

/* Where the route goes: a point IG_LOOKAHEAD along it, if it runs far
 * enough. */
static int IG_RouteAhead(bot_state_t *bs, bot_goal_t *goal, vec3_t target)
{
	vec3_t d;

	if (!trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, IG_LOOKAHEAD, target)) return 0;
	VectorSubtract(target, bs->origin, d);
	d[2] = 0;
	return VectorLength(d) >= IG_MIN_RUN;
}

/* For a chain already moving: the route a short way ahead, somewhere the bot's
 * speed is already taking it (the long straight run IG_RouteAhead asks for is
 * for starting one). */
static int IG_RouteNear(bot_state_t *bs, bot_goal_t *goal, vec3_t target)
{
	vec3_t d, v;
	float speed;

	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	speed = VectorNormalize(v);
	if (speed < 200) return 0;
	if (!trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, 300, target)) return 0;
	VectorSubtract(target, bs->origin, d);
	d[2] = 0;
	if (VectorNormalize(d) < 100) return 0;
	return DotProduct(d, v) > 0.5f;
}

/* An anchor ahead toward `target`, from where the bot is now. */
static int IG_FindAnchorYaw(bot_state_t *bs, float baseyaw, vec3_t anchor)
{
	/* Billy's slingshot: the FLOOR ahead, 10-22 degrees down, straight the way
	 * the bot is moving -- so it only tips its view down, never has to turn
	 * (sideways and overhead anchors, which it could not turn to in time,
	 * are the combat hook's and the climbs'). His middle first. */
	/* shallow: Billy's anchors sit a median 275 units out, 12 degrees down;
	 * the 24-30 degree ones landed under 200 units -- no slingshot at all */
	static const float pitches[] = { 12, 9, 15, 6, 18 };
	static const float yaws[] = { 0, 12, -12, 25, -25 };
	vec3_t dir, flat, end, mid, angles;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d;
	int i, k;

	for (k = 0; k < 5; k++) {
		for (i = 0; i < 5; i++) {
			VectorSet(angles, pitches[i], baseyaw + yaws[k], 0);
			AngleVectors(angles, dir, NULL, NULL);
			VectorSet(flat, dir[0], dir[1], 0);
			VectorNormalize(flat);
			VectorMA(bs->eye, IG_ANCHOR_MAX, dir, end);
			if (!IG_AnchorTrace(bs, end, &tr)) continue;
			d = tr.fraction * IG_ANCHOR_MAX;
			if (d < IG_ANCHOR_MIN) continue;
			/* floor first; a wall straight ahead does too when it is far enough
			 * off that the short pull ends long before it (corridor maps: the
			 * line down hit the end wall before the floor 9 of 10 times) */
			if (tr.plane.normal[2] < 0.7f && (tr.plane.normal[2] < -0.3f || d < 250)) continue;
			/* the body's path to the release point must be clear: level, not
			 * along the hook line -- a floor anchor's line runs into the floor,
			 * and the pull carries the body forward over it */
			VectorMA(bs->origin, IG_RELEASE_FRAC * d, flat, mid);
			BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
			if (tr.fraction < 1) continue;
			/* safe floor all the way the speed will carry it -- and, with
			 * IG-29 on, on the walkway along the line the pull throws it */
			VectorMA(bs->eye, d, dir, anchor);
			if (!IG_WalkwayCorridor(bs, flat, d + 300, anchor)) continue;
			return 1;
		}
	}
	return 0;
}

/* along the way it moves, then toward the route ahead: in a bend, the next
 * hook of a chain is round the corner (chains ended there for want of one) */
static int IG_FindAnchor(bot_state_t *bs, const vec3_t target, vec3_t anchor)
{
	vec3_t v, angles;
	float velyaw = bs->viewangles[YAW], routeyaw;

	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	if (VectorNormalize(v) > 100) {
		vectoangles(v, angles);
		velyaw = angles[YAW];
	}
	if (IG_FindAnchorYaw(bs, velyaw, anchor)) return 1;
	VectorSubtract(target, bs->origin, v);
	v[2] = 0;
	if (VectorNormalize(v) < 64) return 0;
	vectoangles(v, angles);
	routeyaw = angles[YAW];
	if (fabs(AngleDifference(routeyaw, velyaw)) < 20) return 0;     /* the same search */
	return IG_FindAnchorYaw(bs, routeyaw, anchor);
}

/* Is p ahead of the bot, the way it is moving? */
static int IG_InFront(bot_state_t *bs, const vec3_t p)
{
	vec3_t d, v;

	VectorSubtract(p, bs->eye, d);
	d[2] = 0;
	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	if (VectorNormalize(d) < 1 || VectorNormalize(v) < 1) return 1;
	return DotProduct(d, v) > 0.5f;
}

/* IG-20, Billy's stairwell trick: going up stairs, hook the side wall (or the
 * wall at the top) up ahead and slingshot up the stairwell. Upward anchors,
 * 12-36 degrees up, straight on or up to 35 degrees to either side, 200-650
 * units out, hit above the top of the climb (so the pull lifts the bot over
 * it), with the body's path along the pull clear of the steps. */
#define IG_STAIR_MIN_RISE  48
#define IG_STAIR_MAX_RISE  320
#define IG_STAIR_PULL      0.5f   /* seconds pulled at most */
static int IG_StairsOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_stairhook");
}

static int IG_FindStairAnchor(bot_state_t *bs, const vec3_t target, float topz, vec3_t anchor)
{
	static const float pitches[] = { -20, -12, -28, -36 };
	static const float yaws[] = { 0, 20, -20, 35, -35 };
	vec3_t dir, end, mid, angles, flat;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d, baseyaw;
	int i, k;

	VectorSubtract(target, bs->origin, flat);
	flat[2] = 0;
	if (VectorNormalize(flat) < 64) return 0;
	vectoangles(flat, angles);
	baseyaw = angles[YAW];
	for (k = 0; k < 5; k++) {
		for (i = 0; i < 4; i++) {
			VectorSet(angles, pitches[i], baseyaw + yaws[k], 0);
			AngleVectors(angles, dir, NULL, NULL);
			VectorMA(bs->eye, 650, dir, end);
			if (!IG_AnchorTrace(bs, end, &tr)) continue;
			d = tr.fraction * 650;
			if (d < 200) continue;
			if (tr.endpos[2] < topz + 24) continue;            /* above the top of the climb */
			if (tr.plane.normal[2] < -0.3f) continue;          /* not a ceiling */
			VectorMA(bs->origin, 0.6f * d, dir, mid);
			BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
			if (tr.fraction < 1 || tr.startsolid) continue;     /* the steps in the way */
			VectorMA(bs->eye, d, dir, anchor);
			return 1;
		}
	}
	return 0;
}

/* IG-21: hook affinity, a character trait (CHARACTERISTIC_IG_HOOK, 49): 0 a
 * runner, 1 a grapple master (Billy: not every bot plays the same). Missing
 * from a character file it is default_c.c's 9: worked out from the bot's
 * jumping and aggression, which spreads the stock Q3 bots from Phobos (0.38)
 * to Klesk (0.95). IG_HookScale is affinity / 0.55 -- 1 at 0.55, which is
 * how every bot behaved before -- and 1 with the switch (libvar ig_affinity)
 * off. It scales how often the bot chooses to hook, never whether a climb
 * route or the lava escape uses the hook. */
#define CHARACTERISTIC_IG_HOOK  49
#define IG_AFFINITY_MID         0.55f
float Characteristic_Float(int character, int index);
static float IG_HookAffinity(bot_state_t *bs)
{
	float a = Characteristic_Float(bs->character, CHARACTERISTIC_IG_HOOK);

	if (a > 1.5f) {     /* default_c.c's 9: not set (the parser takes no negatives) */
		float j = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_JUMPER, 0, 1);
		float ag = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_AGGRESSION, 0, 1);

		/* floor 0.30 (Billy: fewer runners, more above average): stock bots
		 * 4 runners, 6 average, 16 above, 9 masters */
		a = 0.30f + 0.65f * (0.6f * j + 0.4f * ag);
	}
	if (a < 0.05f) a = 0.05f;
	if (a > 1) a = 1;
	return a;
}

static float IG_HookScale(bot_state_t *bs)
{
	if (!trap_Cvar_VariableIntegerValue("ig_affinity")) return 1;
	return IG_HookAffinity(bs) / IG_AFFINITY_MID;
}

static float IG_Clamp(float v, float lo, float hi)
{
	return v < lo ? lo : v > hi ? hi : v;
}

static void IG_HookMove(bot_state_t *bs, int infight);
static void IG_SlingEnd(ig_sling_t *s)
{
	s->stairs = 0;
	s->state = IG_SLING_IDLE;
	s->cooldown = FloatTime() + IG_COOLDOWN;
}

/* From AINode_Seek_LTG (ai_dmnet.c), after the route's movement and view. */
static void IG_SlingshotStep(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult)
{
	ig_sling_t *s;
	vec3_t target, d, hookpos;
	float speed, dist;
	int c = bs->client;

	if (IG_LavaEscape(bs)) return;
	if (c < 0 || c >= MAX_CLIENTS) return;
	/* travelling, a reposition asked for after a kill or two misses IS a
	 * slingshot: one sooner, rather than the combat hook's sideways pull,
	 * which took the slingshot's turns away (1 204 repositions against 46
	 * slingshots in a test) */
	if (ig_fight[c].urge) {
		ig_fight[c].urge = 0;
		if (ig_sling[c].state == IG_SLING_IDLE) ig_sling[c].cooldown = 0;
	}
	/* a combat hook still finishing: only its own step lets go of it, and a
	 * fight that ended (a kill, a death) left it to travelling, which waited
	 * -- the bot hung until OSP's hook_holdtime, 7.5 s (repositions) */
	if (ig_fight[c].state != IG_FIGHT_NONE) {
		if (ig_fight[c].state == IG_FIGHT_HOOKED) IG_HookMove(bs, 0);
		else ig_fight[c].state = IG_FIGHT_NONE;      /* aiming: drop it */
		return;
	}
	if (!IG_SlingOn()) return;
	s = &ig_sling[c];
	if (s->state != IG_SLING_JUMPED && IG_Hop(bs)) return;
	/* a new map: the clock went back */
	if (s->time > FloatTime() + 1 || s->cooldown > FloatTime() + IG_COOLDOWN + 5) memset(s, 0, sizeof(*s));
	speed = VectorLength(bs->cur_ps.velocity);

	switch (s->state) {
	case IG_SLING_IDLE:
		if (FloatTime() < s->cooldown || !IG_OnGround(bs)) return;
		if ((moveresult->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) return;
		if (moveresult->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) return;
		if (speed < 150 || !IG_RouteAhead(bs, goal, target)) return;
		/* running the way the route goes */
		VectorSubtract(target, bs->origin, d);
		d[2] = 0;
		VectorNormalize(d);
		if (DotProduct(d, bs->cur_ps.velocity) < 0.6f * speed) return;
		/* the jump comes before the anchor, so the way ahead is checked first:
		 * a jump on a walkway beside lava was a jump into it (q2rdm11: lava
		 * deaths 58 -> 157 when the slingshot became jump-first) */
		if (!IG_WalkwayCorridor(bs, d, 500, target)) {
			s->cooldown = FloatTime() + 0.5f;
			return;
		}
		/* how keen this bot is (IG-21): a runner passes on some, a master on none */
		/* IG-28 (libvar ig_affinitychance): the affinity IS the chance, so
		 * every bot above 0.55 no longer takes them all */
		if (random() > (trap_Cvar_VariableIntegerValue("ig_affinity") &&
						trap_Cvar_VariableIntegerValue("ig_affinitychance")
						? IG_HookAffinity(bs)
						: IG_Clamp(1 + (IG_HookScale(bs) - 1) * 0.77f, 0.4f, 1))) {
			s->cooldown = FloatTime() + IG_COOLDOWN * IG_Clamp(2 - IG_HookScale(bs), 0.6f, 2);
			return;
		}
		s->stairs = 0;
		if (IG_StairsOn()) {
			vec3_t up;

			if (trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, 500, up) &&
				up[2] - bs->origin[2] >= IG_STAIR_MIN_RISE && up[2] - bs->origin[2] <= IG_STAIR_MAX_RISE) {
				/* not every time (Billy): a climb is hooked only sometimes
				 * (libvar ig_stairchance, default 35%); otherwise the bot runs
				 * up it, and does not ask again for a couple of seconds */
				char buf[16];
				float chance;

				trap_Cvar_VariableStringBuffer("ig_stairchance", buf, sizeof(buf));
				chance = (buf[0] ? atof(buf) : 0.35f) * IG_HookScale(bs);
				if (random() >= chance) {
					s->cooldown = FloatTime() + 2.5f;
					return;
				}
				s->stairs = 1;
				s->topz = up[2];
				VectorCopy(up, target);
			}
		}
		EA_Jump(c);
		s->state = IG_SLING_JUMPED;
		s->time = FloatTime();
		s->chain = 0;
		return;

	case IG_SLING_JUMPED:
		/* near the top of the jump, from up there, pick the anchor */
		if (FloatTime() - s->time < IG_JUMP_AIM) return;
		if (s->stairs) {
			vec3_t up;

			if (!trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, 500, up) ||
				!IG_FindStairAnchor(bs, up, s->topz, s->anchor)) {
				s->stairs = 0;
				IG_SlingEnd(s);
				return;
			}
			BotAI_Print(PRT_MESSAGE, "ig: stairhook %d rise %.0f\n", c, s->topz - bs->origin[2]);
		}
		else if (!IG_RouteAhead(bs, goal, target)) { IG_SlingEnd(s); return; }
		else if (!IG_FindAnchor(bs, target, s->anchor)) {
			if (s->chain) BotAI_Print(PRT_MESSAGE, "ig: chain end %d noanchor\n", c);
			IG_SlingEnd(s);
			return;
		}
		s->state = IG_SLING_AIMING;
		s->time = FloatTime();
		/* fall through: aim this frame */

	case IG_SLING_AIMING:
		/* the game fires along the view the LAST usercmd carried, so the hook
		 * goes out only on the think after a flick -- and only if that flick
		 * was the last view sent: a think in between spent in another node
		 * (an enemy seen, the battle aim) sent its own view, and hooks fired
		 * then flew level (game pitch -10..6 for 14 asked) */
		if (!IG_FlickReady(bs, s->anchor)) {
			if (FloatTime() - s->time > IG_AIM_TIME) IG_SlingEnd(s);
			return;
		}
		if (FloatTime() - s->time > IG_AIM_TIME) {
			IG_SlingEnd(s);
			return;
		}

		/* the line to the anchor from where the bot is now, not where it was
		 * when it chose it: a blocked or liquid line is a miss */
		{
			bsp_trace_t tr;
			vec3_t far;

			VectorSubtract(s->anchor, bs->eye, d);
			VectorNormalize(d);
			VectorMA(s->anchor, 24, d, far);
			if (!IG_AnchorTrace(bs, far, &tr)) { IG_SlingEnd(s); return; }
			VectorSubtract(tr.endpos, s->anchor, d);
			if (VectorLength(d) > 48) { IG_SlingEnd(s); return; }
		}
		if (!IG_CanFireHook(c)) return;      /* OSP's hook_wait, shared */
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_NoteHookFire(c);
		IG_GrappleTravelNote(c);
		VectorSubtract(s->anchor, bs->eye, d);
		BotAI_Print(PRT_MESSAGE, "ig: slingshot %d length %.0f chain %d air %d\n", c, VectorLength(d), s->chain,
					!IG_OnGround(bs));
		IG_Debug(bs, IG_DBG_CHATTY, "slingshot: %.0f units", VectorLength(d));
		s->state = IG_SLING_HOOKED;
		s->time = FloatTime();
		s->latched = 0;
		s->chain++;
		return;

	case IG_SLING_HOOKED:
		IG_ViewAt(bs, s->anchor);
		IG_GrappleTravelNote(c);        /* ours: the watchdog keeps off */
		/* a hop off the floor while pulled (IG-17): the floor anchor's pull
		 * drags the body into the floor, and there ground friction takes ~60%
		 * of OSP's 975 u/s every frame (the bots were on the floor 41% of the
		 * pull, Billy 20%). A jump that frame clears the ground before the
		 * friction, and the pull goes on at full speed. */
		if (s->latched && IG_OnGround(bs) && trap_Cvar_VariableIntegerValue("ig_pullhop")) {
			vec3_t vd;

			/* only where the floor ahead is safe: a hop carries on past the
			 * pull (q2rdm11 lava deaths 0.24 -> 0.41/min without this) */
			VectorCopy(bs->cur_ps.velocity, vd);
			vd[2] = 0;
			if (VectorNormalize(vd) > 1 && IG_SafeCorridor(bs, vd, 350)) EA_Jump(c);
		}
		/* the flight; the pull counts from the latch */
		if (!s->latched) {
			int st = IG_HookNear(s->anchor, IG_LATCH_RADIUS, hookpos) == 2 && IG_InFront(bs, hookpos);
			vec3_t to, vdir;

			/* or the pull itself: once it catches, OSP sets the velocity to
			 * the pull speed (975) toward the hook, which no run or jump
			 * reaches. A shallow floor hook catches 100-300+ units off the
			 * aimed spot, where the brain did not see it: 18% of slingshots
			 * caught in the game were let go as misses. */
			if (!st && speed > 800) {
				VectorSubtract(s->anchor, bs->eye, to);
				VectorCopy(bs->cur_ps.velocity, vdir);
				to[2] = vdir[2] = 0;
				if (VectorNormalize(to) > 1 && VectorNormalize(vdir) > 1 && DotProduct(to, vdir) > 0.8f) {
					st = 1;
					VectorCopy(s->anchor, hookpos);
				}
			}
			if (st) {
				VectorCopy(hookpos, s->anchor);
				s->latched = FloatTime();
				VectorCopy(bs->origin, s->latchpos);
				VectorSubtract(s->anchor, bs->eye, d);
				s->remain = VectorLength(d);
			}
			else if (FloatTime() - s->time < IG_FLY_MAX) return;
		}
		VectorSubtract(bs->origin, s->latched ? s->latchpos : bs->origin, d);
		dist = VectorLength(d);
		{
			const char *why = !s->latched ? "nolatch" :
				s->stairs ? (bs->origin[2] >= s->topz - 8 ? "top" :
							 FloatTime() - s->latched > IG_STAIR_PULL ? "time" :
							 (FloatTime() - s->latched > 0.15f && speed < IG_STALL_SPEED) ? "stall" : NULL) :
				dist > IG_RELEASE_FRAC * s->remain ? "far" :
				(speed > IG_RELEASE_SPEED && FloatTime() - s->latched >= 0.08f) ? "fast" :   /* at full speed, keep it */
				FloatTime() - s->latched > IG_HOOK_MAX ? "time" :
				(FloatTime() - s->latched > 0.1f && speed < IG_STALL_SPEED) ? "stall" :
				!IG_HookNear(s->anchor, 48, hookpos) ? "lost" : NULL;
			if (!why) return;
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			if (s->stairs) BotAI_Print(PRT_MESSAGE, "ig: stairhook release %d why %s left %.0f\n", c, why, s->topz - bs->origin[2]);
			s->stairs = 0;
			{
				/* where the hook is, if it is out at all: for telling a miss from a refusal */
				int st = IG_HookNear(s->anchor, 4000, hookpos);
				VectorSubtract(hookpos, s->anchor, d);
				BotAI_Print(PRT_MESSAGE, "ig: slingshot release %d speed %.0f after %.0f why %s pulled %.2f hook %d off %.0f\n", c, speed, dist,
							why, s->latched ? FloatTime() - s->latched : 0, st, st ? VectorLength(d) : -1);
			}
			if (s->latched) IG_HopStart(c);
			/* chain, or let it end */
			if (s->latched && random() < IG_Clamp(IG_CHAIN_CHANCE + (IG_HookScale(bs) - 1) * 0.5f, 0.4f, 0.98f)) {
				s->state = IG_SLING_AFTER;
				s->time = FloatTime();
			}
			else IG_SlingEnd(s);
		}
		return;

	case IG_SLING_AFTER:
		/* the next hook of the chain: OSP's wait, then from the air, or a new jump from the ground */
		if (FloatTime() - s->time < IG_REHOOK) return;
		if (FloatTime() - s->time <= IG_CHAIN_WINDOW && !IG_RouteAhead(bs, goal, target) && IG_RouteNear(bs, goal, target)) {
			/* the route bends: a fast bot keeps chaining toward its nearer part
			 * (Billy: more chains, more bots flying down hallways) */
		}
		else if (FloatTime() - s->time > IG_CHAIN_WINDOW || !IG_RouteAhead(bs, goal, target)) {
			BotAI_Print(PRT_MESSAGE, "ig: chain end %d %s\n", c, FloatTime() - s->time > IG_CHAIN_WINDOW ? "window" : "route");
			IG_SlingEnd(s);
			return;
		}
		if (IG_OnGround(bs)) {
			/* the two hops first (IG_Hop, above); a sling only after them */
			if (ig_hops[c] < IG_HOP_MAX && FloatTime() < ig_hopuntil[c]) return;
			VectorSubtract(target, bs->origin, d);
			d[2] = 0;
			VectorNormalize(d);
			if (!IG_WalkwayCorridor(bs, d, 350, target)) {
				BotAI_Print(PRT_MESSAGE, "ig: chain end %d unsafe\n", c);
				IG_SlingEnd(s);
				return;
			}
			BotAI_Print(PRT_MESSAGE, "ig: chain jump %d\n", c);
			EA_Jump(c);
			s->state = IG_SLING_JUMPED;
			s->time = FloatTime();
			return;
		}
		if (!IG_FindAnchor(bs, target, s->anchor)) return;     /* keep looking while in the air */
		s->state = IG_SLING_AIMING;
		s->time = FloatTime();
		IG_ViewAt(bs, s->anchor);
		return;
	}
}

/* ------------------------------------------------------------------------
 * Combat hook (IG-10): a hook mid-fight, for a better angle or to be
 * harder to hit -- what Billy does. From AINode_Battle_Fight, after the bot
 * has aimed and maybe fired (BotCheckAttack).
 *
 * Mostly right after a shot (the railgun's reload is the free moment):
 * with the enemy in view 250-1800 units away and IG_FIGHT_COOLDOWN since
 * the last one, it picks a short anchor (180-450) diagonally sideways and
 * up -- within 55 degrees of where it already looks, because a bot turns no
 * faster than its character lets it -- jumps if on the ground, hooks, rides
 * about a quarter second, lets go, and its aim goes back to the enemy.
 * libvar ig_fighthook 1 (default 0), with the slingshot's hook wiring.
 * Lines "ig: fighthook <bot> ..." are what igbench counts.
 * ------------------------------------------------------------------------ */


static int IG_FightAnchor(bot_state_t *bs, vec3_t anchor, int wide)
{
	/* sideways and up first: height for the angle, sideways to be missed;
	 * a reposition may turn further, it is a move, not a dodge */
	static const float yaws[] = { 40, -40, 55, -55, 25, -25, 75, -75, 90, -90 };
	static const float pitches[] = { 15, 5, 28 };
	vec3_t dir, end, mid, angles;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d, side = random() < 0.5f ? 1 : -1;
	int i, j;

	for (i = 0; i < (wide ? 10 : 6); i++) {
		for (j = 0; j < (int)(sizeof(pitches) / sizeof(pitches[0])); j++) {
			VectorCopy(bs->viewangles, angles);
			angles[YAW] += side * yaws[i];
			angles[PITCH] = -pitches[j];
			AngleVectors(angles, dir, NULL, NULL);
			VectorMA(bs->eye, IG_ANCHOR_MAX, dir, end);
			if (!IG_AnchorTrace(bs, end, &tr)) continue;
			d = tr.fraction * IG_ANCHOR_MAX;
			if (d < IG_ANCHOR_MIN) continue;
			VectorMA(bs->origin, 0.5f * d, dir, mid);
			BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
			if (tr.fraction < 1) continue;
			if (!IG_SafeGround(bs, mid)) continue;
			{
				vec3_t flat;

				VectorCopy(dir, flat);
				flat[2] = 0;
				VectorNormalize(flat);
				VectorMA(bs->eye, d, dir, anchor);
				if (!IG_WalkwayCorridor(bs, flat, d + 200, anchor)) continue;   /* IG-29 */
			}
			VectorMA(bs->eye, d, dir, anchor);
			return 1;
		}
	}
	return 0;
}

static void IG_HookMove(bot_state_t *bs, int infight)
{
	ig_fight_t *f;
	aas_entityinfo_t self, enemy;
	vec3_t d, hookpos;
	float dist, speed;
	int c = bs->client;

	if (IG_LavaEscape(bs)) return;
	if (c < 0 || c >= MAX_CLIENTS || !trap_Cvar_VariableIntegerValue("ig_fighthook") ||
		!trap_Cvar_VariableIntegerValue("ig_hookdetect")) return;
	f = &ig_fight[c];
	if (f->time > FloatTime() + 1 || f->cooldown > FloatTime() + IG_FIGHT_COOLDOWN + 5 ||
		f->shot > FloatTime() + 1 || f->lastkill > FloatTime() + 1) memset(f, 0, sizeof(*f));
	/* shots, hits and kills: a kill sometimes, two misses always, call for a
	 * reposition (Billy: "grapple away to reposition, quickly, and try again") */
	BotEntityInfo(bs->client, &self);
	if (EntityIsShooting(&self) && !f->shooting) f->shot = FloatTime();
	f->shooting = EntityIsShooting(&self);
	if (bs->killedenemy_time > f->lastkill) {
		f->lastkill = bs->killedenemy_time;
		f->shot = 0;
		f->misses = 0;
		if (random() < IG_REPO_AFTER_KILL * IG_HookScale(bs)) f->urge = 1;
	}
	else if (f->shot > 0 && FloatTime() - f->shot > IG_HIT_WINDOW) {
		f->shot = 0;
		if (++f->misses >= IG_REPO_MISSES) {
			f->misses = 0;
			if (random() < IG_HookScale(bs)) f->urge = 1;
		}
	}
	if (ig_sling[c].state != IG_SLING_IDLE && ig_sling[c].state != IG_SLING_AFTER) return;   /* a slingshot is on */
	speed = VectorLength(bs->cur_ps.velocity);

	switch (f->state) {
	case IG_FIGHT_NONE:
		if (IG_InPadFlight(bs)) return;     /* the urge keeps for the landing */
		if (f->urge) {
			/* a reposition: no enemy needed, no cooldown */
			f->urge = 0;
			if (!IG_FightAnchor(bs, f->anchor, 1)) return;
			BotAI_Print(PRT_MESSAGE, "ig: reposition %d\n", c);
		}
		else {
			if (!infight) return;
			if (FloatTime() < f->cooldown || bs->enemy < 0 || bs->enemyvisible_time < FloatTime() - 0.2f) return;
			BotEntityInfo(bs->enemy, &enemy);
			if (!enemy.valid) return;
			VectorSubtract(enemy.origin, bs->origin, d);
			dist = VectorLength(d);
			if (dist < 250 || dist > 1800) return;
			BotEntityInfo(bs->client, &self);
			if (random() > (EntityIsShooting(&self) ? IG_FIGHT_CHANCE : IG_FIGHT_IDLE) * IG_HookScale(bs)) return;
			if (!IG_FightAnchor(bs, f->anchor, 0)) {
				f->cooldown = FloatTime() + 0.5f;
				return;
			}
		}
		if (IG_OnGround(bs)) {
			VectorSubtract(f->anchor, bs->origin, d);
			d[2] = 0;
			VectorNormalize(d);
			if (!IG_SafeCorridor(bs, d, 400)) {
				f->cooldown = FloatTime() + 0.5f;
				return;
			}
			EA_Jump(c);
		}
		f->state = IG_FIGHT_AIMING;
		f->time = FloatTime();
		/* fall through */
	case IG_FIGHT_AIMING:
		if (FloatTime() - f->time > 0.4f) {
			f->state = IG_FIGHT_NONE;
			f->cooldown = FloatTime() + IG_FIGHT_COOLDOWN;
			return;
		}
		if (!IG_FlickReady(bs, f->anchor)) return;
		if (!IG_CanFireHook(c)) return;      /* OSP's hook_wait, shared */
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_NoteHookFire(c);
		IG_GrappleTravelNote(c);
		BotAI_Print(PRT_MESSAGE, "ig: fighthook %d\n", c);
		f->state = IG_FIGHT_HOOKED;
		f->time = FloatTime();
		f->latched = 0;
		return;
	case IG_FIGHT_HOOKED:
		IG_GrappleTravelNote(c);
		if (!f->latched) {
			if (IG_HookNear(f->anchor, IG_LATCH_RADIUS, hookpos) == 2) {
				VectorCopy(hookpos, f->anchor);
				f->latched = FloatTime();
				VectorCopy(bs->origin, f->latchpos);
				VectorSubtract(f->anchor, bs->eye, d);
				f->remain = VectorLength(d);
			}
			else if (FloatTime() - f->time < IG_FLY_MAX) return;
		}
		VectorSubtract(bs->origin, f->latched ? f->latchpos : bs->origin, d);
		dist = VectorLength(d);
		if (f->latched && dist < 0.5f * f->remain && FloatTime() - f->latched < IG_FIGHT_PULL && speed > IG_STALL_SPEED)
			return;     /* still riding it; the AI aims at the enemy meanwhile */
		trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
		BotAI_Print(PRT_MESSAGE, "ig: fighthook release %d latched %d speed %.0f\n", c, f->latched > 0, speed);
		if (f->latched) IG_HopStart(c);
		f->state = IG_FIGHT_NONE;
		f->cooldown = FloatTime() + IG_FIGHT_COOLDOWN;
		return;
	}
}

/* From AINode_Battle_Fight: dodges, and repositions. */
void IG_CombatHook(bot_state_t *bs)
{
	IG_HookMove(bs, 1);
}


/* ------------------------------------------------------------------------
 * Out of the lava (IG-9). A bot in lava or slime hooks nearly straight up
 * -- 85, 75, then 65 degrees; the sky too when the server allows hooking it
 * -- preferring an anchor with safe floor under it (over the walkway, not
 * the pool), on a looser aim than usual (the lava is the hurry). A
 * shallower anchor was the side of a walkway over the lava: out, hanging on
 * the wall, and back in at the let-go (Billy, 2026-11-21). Once out but
 * still over the pool, it lets go and re-hooks toward safe floor. Lets go
 * with safe floor under it, or after 3 s. Overrides a slingshot or a combat
 * hook; called first by IG_Slingshot and IG_CombatHook, so from every node
 * that moves the bot.
 * ------------------------------------------------------------------------ */

enum { IG_LAVA_NONE, IG_LAVA_AIMING, IG_LAVA_HOOKED };
static struct { int state, stage; float time, start, fired; vec3_t anchor; } ig_lava[MAX_CLIENTS];

/* An anchor for the escape: stage 0 out of the pool (steep), stage 1 onto
 * safe floor (shallow, toward it). Best: safe floor under the anchor. */
static int IG_LavaAnchor(bot_state_t *bs, int stage, vec3_t anchor)
{
	static const float yaws[] = { 0, 45, -45, 90, -90, 135, -135, 180 };
	static const float steep[] = { 85, 75, 65, 55, 45 };   /* steep first; 55-45 only if nothing steeper */
	static const float shallow[] = { 35, 20, 5, 50 };
	const float *pitches = stage ? shallow : steep;
	int np = stage ? 4 : 5, i, j, best = 0, score, sky = trap_Cvar_VariableIntegerValue("ig_hooksky");
	vec3_t dir, end, angles, under;
	bsp_trace_t tr;

	for (j = 0; j < np; j++) {
		for (i = 0; i < 8; i++) {
			VectorCopy(bs->viewangles, angles);
			angles[YAW] += yaws[i];
			angles[PITCH] = -pitches[j];
			AngleVectors(angles, dir, NULL, NULL);
			VectorMA(bs->eye, 1500, dir, end);
			if (!IG_AnchorTraceSky(bs, end, &tr, sky) || tr.fraction * 1500 < 64) continue;
			/* safe floor under the anchor, a little back toward the bot (where it hangs) */
			VectorMA(tr.endpos, -48, dir, under);
			score = IG_SafeGround(bs, under) ? 2 : 1;
			if (stage && score < 2) continue;       /* onto floor, or not at all */
			if (score > best) {
				best = score;
				VectorCopy(tr.endpos, anchor);
				if (score == 2) return 1;
			}
		}
	}
	return best > 0;
}

int IG_LavaEscape(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t below;

	if (c < 0 || c >= MAX_CLIENTS || !trap_Cvar_VariableIntegerValue("ig_hookdetect")) return 0;
	if (ig_lava[c].time > FloatTime() + 1) ig_lava[c].state = IG_LAVA_NONE;       /* a new map */
	switch (ig_lava[c].state) {
	case IG_LAVA_NONE:
		if (!BotInLavaOrSlime(bs)) return 0;
		if (!IG_LavaAnchor(bs, 0, ig_lava[c].anchor)) return 0;     /* nothing to hook: the route's own way out */
		ig_sling[c].state = IG_SLING_IDLE;
		ig_fight[c].state = IG_FIGHT_NONE;
		ig_lava[c].state = IG_LAVA_AIMING;
		ig_lava[c].stage = 0;
		ig_lava[c].time = ig_lava[c].start = FloatTime();
		BotAI_Print(PRT_MESSAGE, "ig: lava escape %d\n", c);
		/* fall through */
	case IG_LAVA_AIMING:
		if (FloatTime() - ig_lava[c].start > 3.0f) { ig_lava[c].state = IG_LAVA_NONE; return 1; }
		if (!IG_FlickReady(bs, ig_lava[c].anchor)) return 1;
		if (!IG_CanFireHook(c)) return 1;    /* OSP's hook_wait, shared */
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_NoteHookFire(c);
		IG_GrappleTravelNote(c);
		ig_lava[c].state = IG_LAVA_HOOKED;
		ig_lava[c].time = ig_lava[c].fired = FloatTime();
		return 1;
	case IG_LAVA_HOOKED:
		IG_ViewAt(bs, ig_lava[c].anchor);
		IG_GrappleTravelNote(c);
		VectorCopy(bs->origin, below);
		if (!BotInLavaOrSlime(bs) && IG_SafeGround(bs, below)) {
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			BotAI_Print(PRT_MESSAGE, "ig: lava escape release %d out 1 stage %d\n", c, ig_lava[c].stage);
			ig_lava[c].state = IG_LAVA_NONE;
			return 1;
		}
		/* out of it but hanging over the pool: on to safe floor */
		if (!BotInLavaOrSlime(bs) && ig_lava[c].stage == 0 && FloatTime() - ig_lava[c].time > 0.3f &&
			IG_LavaAnchor(bs, 1, ig_lava[c].anchor)) {
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			ig_lava[c].stage = 1;
			ig_lava[c].state = IG_LAVA_AIMING;
			return 1;
		}
		if (FloatTime() - ig_lava[c].start > 3.0f) {
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			BotAI_Print(PRT_MESSAGE, "ig: lava escape release %d out %d stage %d\n", c, !BotInLavaOrSlime(bs), ig_lava[c].stage);
			ig_lava[c].state = IG_LAVA_NONE;
		}
		return 1;
	}
	return 0;
}

/* ------------------------------------------------------------------------
 * Strafe jumping (IG-11). Quake II caps the speed a player WISHES to move at
 * (300) but not the speed it moves at in the air, where it accelerates by
 * wishspeed * frametime along the wish direction (PM_Accelerate with 1, as
 * pm_airaccelerate is 0). Wishing at the angle whose cosine is
 * (wishspeed - accel) / speed to the velocity adds the most speed each
 * frame -- about 26 degrees at 300, wider as it grows -- and a jump on the
 * landing frame skips the ground's friction. On a long straight stretch of
 * route at running speed the bot jumps, steers so in the air toward the side
 * the route bends (alternating when it runs straight), and jumps again on
 * landing; it stops where the route turns, the floor ahead is not safe, or a
 * hook takes over. libvar ig_strafe 1 (default 0).
 * ------------------------------------------------------------------------ */

#define IG_STRAFE_START  250    /* running at least this fast to start */
#define IG_STRAFE_WISH   300    /* Quake II's pm_maxspeed */
#define IG_STRAFE_DT     0.1f   /* a bot's usercmd covers a server frame */
#define IG_STRAFE_CARVE  (75 * M_PI / 180)  /* IG-31: the most it wishes off its velocity to follow a bend */

static struct { int on; float side; int hops; int stale; vec3_t tdir; } ig_strafe[MAX_CLIENTS];
static float Distance2D(const vec3_t a, const vec3_t b) { return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])); }

static void IG_StrafeJump(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int c = bs->client;
	vec3_t target, tdir, v, wish, perp;
	float speed, cross, cosang, ang, a;

	int mode = trap_Cvar_VariableIntegerValue("ig_strafe");

	if (!mode) return;
	/* IG-31 (ig_strafe 2): a chain already going follows the route a short
	 * way ahead, through its bends, instead of needing a long straight run */
	if (!(mode >= 2 && ig_strafe[c].on && trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, 300, target)) &&
		!IG_RouteAhead(bs, goal, target)) {
		if (ig_strafe[c].on && mode >= 2) BotAI_Print(PRT_MESSAGE, "ig: strafe end %d noroute hops %d\n", c, ig_strafe[c].hops);
		ig_strafe[c].on = 0;
		return;
	}
	VectorSubtract(target, bs->origin, tdir);
	tdir[2] = 0;
	VectorNormalize(tdir);
	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	speed = VectorNormalize(v);
	if (IG_OnGround(bs)) {
		int go;
		/* IG-31: landing just past the route's next point, the route points
		 * back at it; one more hop along the last good direction picks the
		 * route up further on */
		if (mode >= 2 && ig_strafe[c].on && speed >= 250 &&
			(DotProduct(v, tdir) <= 0 || Distance2D(target, bs->origin) < 96) &&
			ig_strafe[c].stale < 1 && DotProduct(v, ig_strafe[c].tdir) > 0.3f &&
			IG_SafeCorridor(bs, ig_strafe[c].tdir, 300)) {
			VectorCopy(ig_strafe[c].tdir, tdir);
			ig_strafe[c].stale++;
		}
		else ig_strafe[c].stale = 0;
		/* a landing mid-run keeps going on looser terms than a start: the
		 * speed is already there and a check failing here threw it away */
		go = ig_strafe[c].on
			? (mode >= 2
				? DotProduct(v, tdir) > 0 && (mr->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK &&
				  !(mr->flags & (MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) && IG_SafeCorridor(bs, tdir, 300)
				: DotProduct(v, tdir) > 0.5f && IG_SafeCorridor(bs, v, 400))
			: (mr->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK &&
			  !(mr->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) &&
			  speed >= IG_STRAFE_START && DotProduct(v, tdir) >= 0.7f && IG_SafeCorridor(bs, tdir, 500);
		if (!go) {
			if (ig_strafe[c].on) BotAI_Print(PRT_MESSAGE, "ig: strafe end %d ground hops %d speed %.0f dot %.2f tdist %.0f travel %d flags %d safe %d\n", c, ig_strafe[c].hops, speed,
				DotProduct(v, tdir), Distance2D(target, bs->origin), mr->traveltype & TRAVELTYPE_MASK, mr->flags, IG_SafeCorridor(bs, tdir, 300));
			ig_strafe[c].on = 0;
			return;
		}
		EA_Jump(c);
		if (!ig_strafe[c].on) { BotAI_Print(PRT_MESSAGE, "ig: strafe %d start %.0f\n", c, speed); ig_strafe[c].hops = 0; }
		else BotAI_Print(PRT_MESSAGE, "ig: strafe hop %d n %d speed %.0f\n", c, ++ig_strafe[c].hops, speed);
		if (!ig_strafe[c].on) ig_strafe[c].stale = 0;
		ig_strafe[c].on = 1;
		if (!ig_strafe[c].stale) VectorCopy(tdir, ig_strafe[c].tdir);
		ig_strafe[c].side = -ig_strafe[c].side;
		if (!ig_strafe[c].side) ig_strafe[c].side = 1;
		return;
	}
	if (!ig_strafe[c].on) return;
	/* IG-31: at strafing speed the bot flies past the route's next point in
	 * the air, and the route then points back at it. Billy keeps going and
	 * picks the route up further on: a point behind or close by keeps the
	 * last good direction until the landing asks again. */
	if (mode >= 2 && speed >= 50) {
		if (DotProduct(v, tdir) > -0.2f && Distance2D(target, bs->origin) >= 96)
			VectorCopy(tdir, ig_strafe[c].tdir);
		else if (VectorLengthSquared(ig_strafe[c].tdir) > 0.5f)
			VectorCopy(ig_strafe[c].tdir, tdir);
	}
	if (mode >= 2 && speed >= 50 && DotProduct(v, tdir) > -0.2f) {
		/* carve: strafe toward the side the route bends to, at the angle that
		 * adds the most speed, or, when the route bends further than that,
		 * at the route itself up to IG_STRAFE_CARVE off the velocity -- Billy
		 * still gains up to 70 degrees off (route8 recording) */
		cross = v[0] * tdir[1] - v[1] * tdir[0];
		ig_strafe[c].side = cross >= 0 ? 1 : -1;
		a = IG_STRAFE_WISH * IG_STRAFE_DT;
		cosang = (IG_STRAFE_WISH - a) / (speed > IG_STRAFE_WISH ? speed : IG_STRAFE_WISH);
		if (cosang > 1) cosang = 1;
		if (cosang < 0) cosang = 0;
		ang = acos(cosang);
		{
			float tang = acos(DotProduct(v, tdir) > 1 ? 1 : DotProduct(v, tdir));
			if (tang > ang) ang = tang < IG_STRAFE_CARVE ? tang : IG_STRAFE_CARVE;
		}
		VectorSet(perp, -v[1], v[0], 0);
		VectorScale(v, cos(ang), wish);
		VectorMA(wish, ig_strafe[c].side * sin(ang), perp, wish);
		EA_Move(c, wish, 400);
		/* coming down: jump now, so it fires on the landing frame (Billy);
		 * the game holds it (ig_botjumpland) */
		if (bs->cur_ps.velocity[2] < 0) EA_Jump(c);
		return;
	}
	if (speed < 50 || DotProduct(v, tdir) < 0.5f) {   /* the route turned */
		BotAI_Print(PRT_MESSAGE, "ig: strafe end %d turned hops %d speed %.0f dot %.2f tdist %.0f vz %.0f tdz %.0f\n", c, ig_strafe[c].hops, speed,
			DotProduct(v, tdir), Distance2D(target, bs->origin), bs->cur_ps.velocity[2], target[2] - bs->origin[2]);
		ig_strafe[c].on = 0;
		return;
	}
	cross = v[0] * tdir[1] - v[1] * tdir[0];
	if (cross > 0.05f) ig_strafe[c].side = 1;
	else if (cross < -0.05f) ig_strafe[c].side = -1;
	a = IG_STRAFE_WISH * IG_STRAFE_DT;
	cosang = (IG_STRAFE_WISH - a) / (speed > IG_STRAFE_WISH ? speed : IG_STRAFE_WISH);
	if (cosang > 1) cosang = 1;
	if (cosang < 0) cosang = 0;
	ang = acos(cosang);
	VectorSet(perp, -v[1], v[0], 0);
	VectorScale(v, cos(ang), wish);
	VectorMA(wish, ig_strafe[c].side * sin(ang), perp, wish);
	EA_Move(c, wish, 400);
}

/* From the route nodes (ai_dmnet.c): the hook techniques first, then, with
 * no hook busy, strafe jumping. */
/* IG-13: the pad hook. Flying a jump pad's arc, the bot hooks the floor just
 * past the landing spot and lets go nearly over it: OSP's pull (975 u/s, a
 * straight line) beats the slow top of the arc (Billy: "when a jump pad is
 * taken, the bot uses the grappling hook to get to the landing spot faster"). */
#define IG_PAD_MIN      120     /* closer than this, the arc is as quick */
#define IG_PAD_LATE     450     /* hook only within this of the landing (horizontally) */
#define IG_PAD_TOPVEL   100     /* ... and near the top of the arc or coming down */
#define IG_PAD_SETTLE   0.15f   /* seconds after the launch before it hooks */
#define IG_PAD_RELEASE  110     /* let go this close (horizontally) to the landing */
#define IG_PAD_MAXPULL  0.35f   /* from the fire: a short pull, then the momentum carries it (Billy) */
enum { IG_PAD_NONE, IG_PAD_AIMING, IG_PAD_HOOKED, IG_PAD_DONE };
static struct { int state; float start, time, dist0; vec3_t anchor, land; } ig_pad[MAX_CLIENTS];

static int IG_PadOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_padhook") && trap_Cvar_VariableIntegerValue("ig_hookdetect");
}

/* The hook point: whatever is behind a spot a little above the landing, on
 * the line from the eye through it -- the far wall, or the floor beyond. The
 * pull then carries the bot over the landing, where it lets go. (The floor
 * at the landing itself is hidden behind the ledge's edge for most of a
 * climb.) */
static int IG_PadAnchor(bot_state_t *bs, const vec3_t land, vec3_t anchor)
{
	vec3_t fwd, aim, end, d;
	bsp_trace_t tr;
	float tolanding;

	VectorSubtract(land, bs->origin, fwd);
	fwd[2] = 0;
	if (VectorNormalize(fwd) < IG_PAD_MIN) return 0;
	/* above the landing, at the end of the arc: the floor just past it, a pull
	 * down and forward onto it (a point above it lifted the bot and landed it
	 * later) */
	if (bs->origin[2] > land[2] + 16) {
		VectorMA(land, 40, fwd, aim);
		aim[2] = land[2] - 24 - 4;
		VectorSubtract(aim, bs->eye, d);
		VectorNormalize(d);
		VectorMA(aim, 24, d, end);
		if (IG_AnchorTrace(bs, end, &tr)) {
			VectorSubtract(tr.endpos, aim, d);
			if (VectorLength(d) < 48) {
				VectorCopy(tr.endpos, anchor);
				return 1;
			}
		}
	}
	/* the whole body must clear the ledge's lip on the way, not only the eye:
	 * pulled along the eye's line, the feet caught the edge and it hung there
	 * (30% of pad hooks pulled a full second without arriving) */
	{
		vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32}, over;

		VectorCopy(land, over);
		over[2] += 64;
		BotAI_Trace(&tr, bs->origin, mins, maxs, over, bs->entitynum, MASK_SOLID);
		if (tr.fraction < 1 || tr.startsolid) return 0;
	}
	VectorCopy(land, aim);
	aim[2] += 64 + bs->cur_ps.viewheight;
	VectorSubtract(aim, bs->eye, d);
	tolanding = VectorNormalize(d);
	VectorMA(bs->eye, tolanding + 600, d, end);
	if (!IG_AnchorTrace(bs, end, &tr)) return 0;
	if (tr.fraction * (tolanding + 600) < tolanding + 24) return 0;   /* blocked short of the landing */
	VectorCopy(tr.endpos, anchor);
	return 1;
}

/* true while it owns the bot's hook and view */
static int IG_PadHook(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t land, d, hookpos;

	if (c < 0 || c >= MAX_CLIENTS || !trap_Cvar_VariableIntegerValue("ig_hookdetect")) return 0;
	if (ig_pad[c].time > FloatTime() + 1) ig_pad[c].state = IG_PAD_NONE;      /* a new map */
	switch (ig_pad[c].state) {
	case IG_PAD_DONE:
		if (!IG_OnGround(bs)) return 0;
		BotAI_Print(PRT_MESSAGE, "ig: padflight %d hooked %.2f dist %.0f\n", c, FloatTime() - ig_pad[c].start, ig_pad[c].dist0);
		ig_pad[c].state = IG_PAD_NONE;
		ig_pad[c].start = 0;
		return 0;
	case IG_PAD_NONE:
		if (IG_OnGround(bs) || !IG_PadFlight(c, land)) {
			if (ig_pad[c].start > 0 && IG_OnGround(bs))
				BotAI_Print(PRT_MESSAGE, "ig: padflight %d plain %.2f dist %.0f land %.0f %.0f %.0f at %.0f %.0f %.0f\n", c, FloatTime() - ig_pad[c].start, ig_pad[c].dist0, ig_pad[c].land[0], ig_pad[c].land[1], ig_pad[c].land[2], bs->origin[0], bs->origin[1], bs->origin[2]);
			ig_pad[c].start = 0;
			return 0;
		}
		if (ig_pad[c].start <= 0) {
			ig_pad[c].start = FloatTime();
			VectorCopy(land, ig_pad[c].land);
			VectorSubtract(land, bs->origin, d);
			d[2] = 0;
			ig_pad[c].dist0 = VectorLength(d);
		}
		if (!IG_PadOn()) return 0;
		if (FloatTime() - ig_pad[c].start < IG_PAD_SETTLE) return 0;
		/* the end of the arc, just before the landing: near the top or on
		 * the way down, and close to it (Billy: "just before the landing,
		 * a slight advantage in speed, towards the end of the arc") */
		if (bs->cur_ps.velocity[2] > IG_PAD_TOPVEL) return 0;
		VectorSubtract(land, bs->origin, d);
		d[2] = 0;
		if (VectorLength(d) > IG_PAD_LATE) return 0;
		if (ig_sling[c].state != IG_SLING_IDLE && ig_sling[c].state != IG_SLING_AFTER) return 0;
		if (ig_fight[c].state != IG_FIGHT_NONE) return 0;
		if (!IG_PadAnchor(bs, land, ig_pad[c].anchor)) return 0;
		if (random() > IG_HookScale(bs)) { ig_pad[c].state = IG_PAD_DONE; return 0; }     /* IG-21: this flight, no */
		VectorCopy(land, ig_pad[c].land);
		ig_pad[c].state = IG_PAD_AIMING;
		ig_pad[c].time = FloatTime();
		/* fall through */
	case IG_PAD_AIMING:
		if (IG_OnGround(bs) || FloatTime() - ig_pad[c].time > 0.5f) {
			ig_pad[c].state = IG_PAD_DONE;
			return 0;
		}
		if (!IG_FlickReady(bs, ig_pad[c].anchor)) return 1;
		if (!IG_CanFireHook(c)) return 1;
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_NoteHookFire(c);
		IG_GrappleTravelNote(c);
		VectorSubtract(ig_pad[c].land, bs->origin, d);
		BotAI_Print(PRT_MESSAGE, "ig: padhook %d dist %.0f\n", c, VectorLength(d));
		ig_pad[c].state = IG_PAD_HOOKED;
		ig_pad[c].time = FloatTime();
		return 1;
	case IG_PAD_HOOKED:
		IG_GrappleTravelNote(c);
		VectorSubtract(ig_pad[c].land, bs->origin, d);
		d[2] = 0;
		{
			const char *why =
				VectorLength(d) < IG_PAD_RELEASE && bs->origin[2] > ig_pad[c].land[2] - 32 ? "near" :
				IG_OnGround(bs) ? "ground" :
				FloatTime() - ig_pad[c].time > IG_PAD_MAXPULL ? "time" :
				(FloatTime() - ig_pad[c].time > 0.5f && !IG_HookNear(ig_pad[c].anchor, 300, hookpos)) ? "lost" : NULL;
			if (!why) {
				/* steer for the landing while pulled */
				VectorNormalize(d);
				EA_Move(c, d, 400);
				return 1;
			}
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			BotAI_Print(PRT_MESSAGE, "ig: padhook release %d why %s after %.2f left %.0f\n", c, why,
						FloatTime() - ig_pad[c].time, VectorLength(d));
			ig_pad[c].state = IG_PAD_DONE;
			return 0;
		}
	}
	return 0;
}

/* IG-13, the first half of Billy's pad trick: running at a jump pad on the
 * route, jump, hook the pad, and let go just before it -- still hooked, OSP's
 * pull would override the pad's launch. The momentum carries the bot onto the
 * trigger, sooner than running there. */
#define IG_PADIN_MIN     200
#define IG_PADIN_MAX     700
#define IG_PADIN_LETGO   90     /* let go this far (horizontally) short of the pad */
enum { IG_PADIN_NONE, IG_PADIN_JUMPED, IG_PADIN_AIMING, IG_PADIN_HOOKED };
static struct { int state; float time, cooldown; vec3_t pad, anchor; } ig_padin[MAX_CLIENTS];

/* the jump pad the route takes within the next couple of seconds */
static int IG_PadAhead(bot_state_t *bs, bot_goal_t *goal, vec3_t pad)
{
	aas_predictroute_t route;

	if (!goal || goal->areanum <= 0 || bs->areanum <= 0) return 0;
	trap_AAS_PredictRoute(&route, bs->areanum, bs->origin, goal->areanum, bs->tfl, 30, 250,
						  RSE_USETRAVELTYPE, 0, TFL_JUMPPAD, 0);
	if (!(route.stopevent & RSE_USETRAVELTYPE)) return 0;
	VectorCopy(route.endpos, pad);
	return 1;
}

static int IG_PadApproach(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int c = bs->client;
	vec3_t pad, d, aim, end, hookpos;
	bsp_trace_t tr;
	float dist;

	if (c < 0 || c >= MAX_CLIENTS || !IG_PadOn()) return 0;
	if (ig_padin[c].time > FloatTime() + 1 || ig_padin[c].cooldown > FloatTime() + 10) memset(&ig_padin[c], 0, sizeof(ig_padin[c]));
	switch (ig_padin[c].state) {
	case IG_PADIN_NONE:
		if (FloatTime() < ig_padin[c].cooldown || !IG_OnGround(bs)) return 0;
		if (ig_sling[c].state != IG_SLING_IDLE || ig_fight[c].state != IG_FIGHT_NONE) return 0;
		if ((mr->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) return 0;
		if (!IG_PadAhead(bs, goal, pad)) return 0;
		VectorSubtract(pad, bs->origin, d);
		if (fabs(d[2]) > 64) return 0;
		d[2] = 0;
		dist = VectorNormalize(d);
		if (dist < IG_PADIN_MIN || dist > IG_PADIN_MAX) return 0;
		if (!IG_SafeCorridor(bs, d, dist)) { ig_padin[c].cooldown = FloatTime() + 1; return 0; }
		if (random() > IG_HookScale(bs)) { ig_padin[c].cooldown = FloatTime() + 3; return 0; }     /* IG-21 */
		VectorCopy(pad, ig_padin[c].pad);
		EA_Jump(c);
		ig_padin[c].state = IG_PADIN_JUMPED;
		ig_padin[c].time = FloatTime();
		return 1;
	case IG_PADIN_JUMPED:
		if (FloatTime() - ig_padin[c].time < 0.1f) return 1;
		/* the pad's floor, a little past where the route steps on */
		VectorSubtract(ig_padin[c].pad, bs->origin, d);
		d[2] = 0;
		VectorNormalize(d);
		VectorMA(ig_padin[c].pad, 32, d, aim);
		aim[2] = ig_padin[c].pad[2] - 24 - 4;
		VectorSubtract(aim, bs->eye, d);
		VectorNormalize(d);
		VectorMA(aim, 24, d, end);
		if (!IG_AnchorTrace(bs, end, &tr)) { ig_padin[c].state = IG_PADIN_NONE; ig_padin[c].cooldown = FloatTime() + 1; return 0; }
		VectorSubtract(tr.endpos, aim, d);
		if (VectorLength(d) > 48) { ig_padin[c].state = IG_PADIN_NONE; ig_padin[c].cooldown = FloatTime() + 1; return 0; }
		VectorCopy(tr.endpos, ig_padin[c].anchor);
		ig_padin[c].state = IG_PADIN_AIMING;
		ig_padin[c].time = FloatTime();
		/* fall through */
	case IG_PADIN_AIMING:
		if (FloatTime() - ig_padin[c].time > 0.5f) { ig_padin[c].state = IG_PADIN_NONE; ig_padin[c].cooldown = FloatTime() + 1; return 0; }
		if (!IG_FlickReady(bs, ig_padin[c].anchor)) return 1;
		if (!IG_CanFireHook(c)) return 1;
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_NoteHookFire(c);
		IG_GrappleTravelNote(c);
		VectorSubtract(ig_padin[c].pad, bs->origin, d);
		d[2] = 0;
		BotAI_Print(PRT_MESSAGE, "ig: padin %d dist %.0f\n", c, VectorLength(d));
		ig_padin[c].state = IG_PADIN_HOOKED;
		ig_padin[c].time = FloatTime();
		return 1;
	case IG_PADIN_HOOKED:
		IG_GrappleTravelNote(c);
		VectorSubtract(ig_padin[c].pad, bs->origin, d);
		d[2] = 0;
		dist = VectorLength(d);
		{
			const char *why = dist < IG_PADIN_LETGO ? "near" :
				FloatTime() - ig_padin[c].time > 0.8f ? "time" :
				(FloatTime() - ig_padin[c].time > 0.5f && !IG_HookNear(ig_padin[c].anchor, 200, hookpos)) ? "lost" : NULL;
			if (!why) {
				VectorNormalize(d);
				EA_Move(c, d, 400);
				return 1;
			}
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			BotAI_Print(PRT_MESSAGE, "ig: padin release %d why %s after %.2f left %.0f\n", c, why, FloatTime() - ig_padin[c].time, dist);
			ig_padin[c].state = IG_PADIN_NONE;
			ig_padin[c].cooldown = FloatTime() + 1.5f;
			/* keep running onto the pad: the route does it */
			return 0;
		}
	}
	return 0;
}

void IG_Slingshot(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult)
{
	int c = bs->client;

	if (IG_PadHook(bs)) return;      /* IG-13: flying a pad, hooking its landing */
	if (c >= 0 && c < MAX_CLIENTS && IG_InPadFlight(bs)) {
		/* a chain ends at a pad; the pad carries the bot from here */
		if (ig_sling[c].state != IG_SLING_IDLE && ig_sling[c].state != IG_SLING_HOOKED) IG_SlingEnd(&ig_sling[c]);
		ig_strafe[c].on = 0;
		if (ig_sling[c].state == IG_SLING_IDLE) return;
	}
	if (IG_PadApproach(bs, goal, moveresult)) return;     /* IG-13: hooking INTO a pad */
	IG_SlingshotStep(bs, goal, moveresult);
	if (c < 0 || c >= MAX_CLIENTS) return;
	/* steering is the move, not the view: it goes on while a slingshot jumps
	 * and aims, and pauses only while a hook pulls */
	if (ig_sling[c].state == IG_SLING_HOOKED || ig_fight[c].state != IG_FIGHT_NONE ||
		ig_lava[c].state != IG_LAVA_NONE || BotInLavaOrSlime(bs)) {
		if (ig_sling[c].state != IG_SLING_HOOKED) ig_strafe[c].on = 0;
		return;
	}
	if (ig_sling[c].state != IG_SLING_IDLE && IG_OnGround(bs)) return;     /* the slingshot's own jump */
	IG_StrafeJump(bs, goal, moveresult);
}
