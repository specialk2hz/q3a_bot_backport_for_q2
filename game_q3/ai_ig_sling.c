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

/* botlib/be_ig_hook.c */
void IG_GrappleTravelNote(int client);
void IG_NoteHookFire(int client);
int  IG_CanFireHook(int client);
int  IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void EA_Jump(int client);

#define IG_LOOKAHEAD     700    /* how far along the route it looks */
#define IG_MIN_RUN       250    /* the route must run on this far, horizontally */
#define IG_ANCHOR_MIN    180
#define IG_ANCHOR_MAX    700
#define IG_JUMP_AIM      0.15f  /* seconds into the jump to pick the anchor */
#define IG_AIM_TIME      0.5f   /* seconds to get aimed and fire */
#define IG_AIM_TOL       4      /* degrees: looser was sloppy -- misses that drop the bot */
#define IG_FLY_MAX       0.4f   /* seconds for the hook to latch (a short throw) */
#define IG_RELEASE_FRAC  0.6f   /* let go this far along the pull ... */
#define IG_RELEASE_SPEED 900    /* ... or this fast, after a moment's pull */
#define IG_HOOK_MAX      0.35f  /* seconds pulled at most, from the latch */
#define IG_STALL_SPEED   150    /* pulled but this slow: the body is caught */
#define IG_LATCH_RADIUS  650    /* how far from the aimed anchor the hook may latch: a
                                  * floor hook at a shallow angle lands far along the
                                  * floor for a few degrees of aim; it is the bot's
                                  * hook if it is in front of it (IG_InFront) */
#define IG_REHOOK        0.5f   /* OSP's hook_wait: the soonest it can hook again */
#define IG_CHAIN_WINDOW  2.5f   /* in the air this long after a let-go, it may chain */
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
#define IG_HOP_MAX    3
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

static int IG_AimedNow(bot_state_t *bs, float tol)
{
	return fabs(AngleDifference(bs->viewangles[0], bs->ideal_viewangles[0])) < tol &&
		   fabs(AngleDifference(bs->viewangles[1], bs->ideal_viewangles[1])) < tol;
}

/* Aimed, and already aimed the think before. A client command (the hook)
 * runs at once, with the view the GAME has -- the last usercmd's --, while
 * the brain's view is this think's: firing the think it first got aimed
 * sent the hook along last frame's view, past the anchor (63% of the
 * slingshots that never latched). */
static float ig_aimedsince[MAX_CLIENTS];
static int IG_AimedFor(bot_state_t *bs, float tol)
{
	int c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS) return 0;
	if (!IG_AimedNow(bs, tol)) {
		ig_aimedsince[c] = 0;
		return 0;
	}
	if (ig_aimedsince[c] <= 0 || ig_aimedsince[c] > FloatTime()) {
		ig_aimedsince[c] = FloatTime();
		return 0;
	}
	return FloatTime() > ig_aimedsince[c];
}

static int IG_Aimed(bot_state_t *bs)
{
	return IG_AimedFor(bs, IG_AIM_TOL);
}

static int IG_OnGround(bot_state_t *bs)
{
	return bs->cur_ps.groundEntityNum != ENTITYNUM_NONE;
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

int IG_LavaEscape(bot_state_t *bs);
static void IG_RepositionHook(bot_state_t *bs);

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

/* An anchor ahead toward `target`, from where the bot is now. */
static int IG_FindAnchor(bot_state_t *bs, const vec3_t target, vec3_t anchor)
{
	/* above and below eye level, Billy's middle first */
	static const float pitches[] = { -12, -4, -18, 4, 12 };
	vec3_t dir, flat, end, mid, angles;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d;
	int i;

	/* straight along the route first, then up to 30 degrees either side: a
	 * person glances for the nearest wall to take (chains ended for want of
	 * an anchor dead ahead: 654 of 2 100 in a test) */
	static const float yaws[] = { 0, 15, -15, 30, -30 };
	float routeyaw;
	int k;

	VectorSubtract(target, bs->eye, flat);
	flat[2] = 0;
	if (VectorNormalize(flat) < IG_ANCHOR_MIN) return 0;
	vectoangles(flat, angles);
	routeyaw = angles[YAW];
	for (k = 0; k < 25; k++) {
		i = k % 5;
		angles[YAW] = routeyaw + yaws[k / 5];
		angles[PITCH] = -pitches[i];
		AngleVectors(angles, dir, NULL, NULL);
		AngleVectors(angles, flat, NULL, NULL);
		flat[2] = 0;
		VectorNormalize(flat);
		VectorMA(bs->eye, IG_ANCHOR_MAX, dir, end);
		if (!IG_AnchorTrace(bs, end, &tr)) continue;
		d = tr.fraction * IG_ANCHOR_MAX;
		if (d < IG_ANCHOR_MIN) continue;
		/* the body's path to the release point must be clear */
		VectorMA(bs->origin, IG_RELEASE_FRAC * d, dir, mid);
		BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID | IG_LIQUIDS);
		if (tr.fraction < 1) continue;
		/* safe floor (not a pit, not lava) under the let-go, the anchor, and
		 * beyond, where the speed it keeps carries it */
		if (!IG_SafeGround(bs, mid)) continue;
		if (!IG_SafeCorridor(bs, flat, d + 300)) continue;
		VectorMA(bs->eye, d, dir, anchor);
		return 1;
	}
	return 0;
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

static void IG_SlingEnd(ig_sling_t *s)
{
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
	IG_RepositionHook(bs);
	if (c >= 0 && c < MAX_CLIENTS && ig_fight[c].state != IG_FIGHT_NONE) return;   /* a reposition is on */
	if (c < 0 || c >= MAX_CLIENTS || !IG_SlingOn()) return;
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
		if (!IG_SafeCorridor(bs, d, 500)) {
			s->cooldown = FloatTime() + 0.5f;
			return;
		}
		EA_Jump(c);
		s->state = IG_SLING_JUMPED;
		s->time = FloatTime();
		s->chain = 0;
		return;

	case IG_SLING_JUMPED:
		/* near the top of the jump, from up there, pick the anchor */
		if (FloatTime() - s->time < IG_JUMP_AIM) return;
		if (!IG_RouteAhead(bs, goal, target) || !IG_FindAnchor(bs, target, s->anchor)) {
			if (s->chain) BotAI_Print(PRT_MESSAGE, "ig: chain end %d noanchor\n", c);
			IG_SlingEnd(s);
			return;
		}
		s->state = IG_SLING_AIMING;
		s->time = FloatTime();
		/* fall through: aim this frame */

	case IG_SLING_AIMING:
		IG_ViewAt(bs, s->anchor);
		if (FloatTime() - s->time > IG_AIM_TIME) {
			IG_SlingEnd(s);
			return;
		}
		if (!IG_Aimed(bs)) return;
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
		/* the flight; the pull counts from the latch */
		if (!s->latched) {
			if (IG_HookNear(s->anchor, IG_LATCH_RADIUS, hookpos) == 2 && IG_InFront(bs, hookpos)) {
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
				dist > IG_RELEASE_FRAC * s->remain ? "far" :
				(speed > IG_RELEASE_SPEED && FloatTime() - s->latched >= 0.08f) ? "fast" :   /* at full speed, keep it */
				FloatTime() - s->latched > IG_HOOK_MAX ? "time" :
				(FloatTime() - s->latched > 0.1f && speed < IG_STALL_SPEED) ? "stall" :
				!IG_HookNear(s->anchor, 48, hookpos) ? "lost" : NULL;
			if (!why) return;
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			{
				/* where the hook is, if it is out at all: for telling a miss from a refusal */
				int st = IG_HookNear(s->anchor, 4000, hookpos);
				VectorSubtract(hookpos, s->anchor, d);
				BotAI_Print(PRT_MESSAGE, "ig: slingshot release %d speed %.0f after %.0f why %s pulled %.2f hook %d off %.0f\n", c, speed, dist,
							why, s->latched ? FloatTime() - s->latched : 0, st, st ? VectorLength(d) : -1);
			}
			if (s->latched) IG_HopStart(c);
			/* chain, or let it end */
			if (s->latched && random() < IG_CHAIN_CHANCE) {
				s->state = IG_SLING_AFTER;
				s->time = FloatTime();
			}
			else IG_SlingEnd(s);
		}
		return;

	case IG_SLING_AFTER:
		/* the next hook of the chain: OSP's wait, then from the air, or a new jump from the ground */
		if (FloatTime() - s->time < IG_REHOOK) return;
		if (FloatTime() - s->time > IG_CHAIN_WINDOW || !IG_RouteAhead(bs, goal, target)) {
			BotAI_Print(PRT_MESSAGE, "ig: chain end %d %s\n", c, FloatTime() - s->time > IG_CHAIN_WINDOW ? "window" : "route");
			IG_SlingEnd(s);
			return;
		}
		if (IG_OnGround(bs)) {
			VectorSubtract(target, bs->origin, d);
			d[2] = 0;
			VectorNormalize(d);
			if (!IG_SafeCorridor(bs, d, 500)) {
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
				if (!IG_SafeCorridor(bs, flat, d + 200)) continue;
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
		if (random() < IG_REPO_AFTER_KILL) f->urge = 1;
	}
	else if (f->shot > 0 && FloatTime() - f->shot > IG_HIT_WINDOW) {
		f->shot = 0;
		if (++f->misses >= IG_REPO_MISSES) {
			f->misses = 0;
			f->urge = 1;
		}
	}
	if (ig_sling[c].state != IG_SLING_IDLE && ig_sling[c].state != IG_SLING_AFTER) return;   /* a slingshot is on */
	speed = VectorLength(bs->cur_ps.velocity);

	switch (f->state) {
	case IG_FIGHT_NONE:
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
			if (random() > (EntityIsShooting(&self) ? IG_FIGHT_CHANCE : IG_FIGHT_IDLE)) return;
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
		IG_ViewAt(bs, f->anchor);
		if (FloatTime() - f->time > 0.4f) {
			f->state = IG_FIGHT_NONE;
			f->cooldown = FloatTime() + IG_FIGHT_COOLDOWN;
			return;
		}
		if (!IG_Aimed(bs)) return;
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

/* From the route nodes (via IG_Slingshot): a reposition called for after a
 * kill -- the fight node is left the moment the enemy dies. */
static void IG_RepositionHook(bot_state_t *bs)
{
	IG_HookMove(bs, 0);
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
		IG_ViewAt(bs, ig_lava[c].anchor);
		if (FloatTime() - ig_lava[c].start > 3.0f) { ig_lava[c].state = IG_LAVA_NONE; return 1; }
		if (!IG_AimedFor(bs, 15)) return 1;
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

static struct { int on; float side; int hops; } ig_strafe[MAX_CLIENTS];

static void IG_StrafeJump(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int c = bs->client;
	vec3_t target, tdir, v, wish, perp;
	float speed, cross, cosang, ang, a;

	if (!trap_Cvar_VariableIntegerValue("ig_strafe")) return;
	if (!IG_RouteAhead(bs, goal, target)) { ig_strafe[c].on = 0; return; }
	VectorSubtract(target, bs->origin, tdir);
	tdir[2] = 0;
	VectorNormalize(tdir);
	VectorCopy(bs->cur_ps.velocity, v);
	v[2] = 0;
	speed = VectorNormalize(v);
	if (IG_OnGround(bs)) {
		/* a landing mid-run keeps going on looser terms than a start: the
		 * speed is already there and a check failing here threw it away */
		int go = ig_strafe[c].on
			? DotProduct(v, tdir) > 0.5f && IG_SafeCorridor(bs, v, 400)
			: (mr->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK &&
			  !(mr->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) &&
			  speed >= IG_STRAFE_START && DotProduct(v, tdir) >= 0.7f && IG_SafeCorridor(bs, tdir, 500);
		if (!go) {
			if (ig_strafe[c].on) BotAI_Print(PRT_MESSAGE, "ig: strafe end %d ground hops %d speed %.0f\n", c, ig_strafe[c].hops, speed);
			ig_strafe[c].on = 0;
			return;
		}
		EA_Jump(c);
		if (!ig_strafe[c].on) { BotAI_Print(PRT_MESSAGE, "ig: strafe %d start %.0f\n", c, speed); ig_strafe[c].hops = 0; }
		else BotAI_Print(PRT_MESSAGE, "ig: strafe hop %d n %d speed %.0f\n", c, ++ig_strafe[c].hops, speed);
		ig_strafe[c].on = 1;
		ig_strafe[c].side = -ig_strafe[c].side;
		if (!ig_strafe[c].side) ig_strafe[c].side = 1;
		return;
	}
	if (!ig_strafe[c].on) return;
	if (speed < 50 || DotProduct(v, tdir) < 0.5f) {   /* the route turned */
		BotAI_Print(PRT_MESSAGE, "ig: strafe end %d turned hops %d speed %.0f\n", c, ig_strafe[c].hops, speed);
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
void IG_Slingshot(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult)
{
	int c = bs->client;

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
