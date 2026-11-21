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
int  IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void EA_Jump(int client);

#define IG_LOOKAHEAD     700    /* how far along the route it looks */
#define IG_MIN_RUN       400    /* the route must run on this far, horizontally */
#define IG_ANCHOR_MIN    180
#define IG_ANCHOR_MAX    450
#define IG_JUMP_AIM      0.15f  /* seconds into the jump to pick the anchor */
#define IG_AIM_TIME      0.5f   /* seconds to get aimed and fire */
#define IG_AIM_TOL       8      /* degrees */
#define IG_FLY_MAX       0.4f   /* seconds for the hook to latch (a short throw) */
#define IG_RELEASE_FRAC  0.6f   /* let go this far along the pull ... */
#define IG_RELEASE_SPEED 850    /* ... or this fast and past a quarter of it */
#define IG_HOOK_MAX      0.35f  /* seconds pulled at most, from the latch */
#define IG_STALL_SPEED   150    /* pulled but this slow: the body is caught */
#define IG_LATCH_RADIUS  450    /* how far from the aimed anchor the hook may latch: a
                                  * floor hook at a shallow angle lands far along the
                                  * floor for a few degrees of aim; it is the bot's
                                  * hook if it is in front of it (IG_InFront) */
#define IG_REHOOK        0.5f   /* OSP's hook_wait: the soonest it can hook again */
#define IG_CHAIN_WINDOW  1.5f   /* in the air this long after a let-go, it may chain */
#define IG_CHAIN_CHANCE  0.6f   /* how often it does (Billy: 59%) */
#define IG_COOLDOWN      1.5f   /* after a sequence ends, before the next jump */

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

static int IG_Aimed(bot_state_t *bs)
{
	return fabs(AngleDifference(bs->viewangles[0], bs->ideal_viewangles[0])) < IG_AIM_TOL &&
		   fabs(AngleDifference(bs->viewangles[1], bs->ideal_viewangles[1])) < IG_AIM_TOL;
}

static int IG_OnGround(bot_state_t *bs)
{
	return bs->cur_ps.groundEntityNum != ENTITYNUM_NONE;
}

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
	vec3_t dir, flat, end, mid, down, angles;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d;
	int i;

	VectorSubtract(target, bs->eye, flat);
	flat[2] = 0;
	if (VectorNormalize(flat) < IG_ANCHOR_MIN) return 0;
	vectoangles(flat, angles);
	for (i = 0; i < (int)(sizeof(pitches) / sizeof(pitches[0])); i++) {
		angles[PITCH] = -pitches[i];
		AngleVectors(angles, dir, NULL, NULL);
		VectorMA(bs->eye, IG_ANCHOR_MAX, dir, end);
		BotAI_Trace(&tr, bs->eye, NULL, NULL, end, bs->entitynum, MASK_SOLID);
		if (tr.fraction >= 1 || tr.startsolid) continue;
		if (tr.surface.flags & SURF_SKY) continue;     /* the bots do not hook the sky (yet) */
		d = tr.fraction * IG_ANCHOR_MAX;
		if (d < IG_ANCHOR_MIN) continue;
		/* the body's path to the release point must be clear */
		VectorMA(bs->origin, IG_RELEASE_FRAC * d, dir, mid);
		BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID);
		if (tr.fraction < 1) continue;
		/* and there must be floor (not a pit, not lava) under where it flies on */
		VectorMA(bs->origin, d, flat, mid);
		VectorCopy(mid, down);
		down[2] -= 700;
		BotAI_Trace(&tr, mid, NULL, NULL, down, bs->entitynum, MASK_SOLID | CONTENTS_LAVA | CONTENTS_SLIME);
		if (tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME))) continue;
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
void IG_Slingshot(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult)
{
	ig_sling_t *s;
	vec3_t target, d, hookpos;
	float speed, dist;
	int c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS || !IG_SlingOn()) return;
	s = &ig_sling[c];
	/* a new map: the clock went back */
	if (s->time > FloatTime() + 1 || s->cooldown > FloatTime() + IG_COOLDOWN + 5) memset(s, 0, sizeof(*s));
	speed = VectorLength(bs->cur_ps.velocity);

	switch (s->state) {
	case IG_SLING_IDLE:
		if (FloatTime() < s->cooldown || !IG_OnGround(bs)) return;
		if ((moveresult->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) return;
		if (moveresult->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) return;
		if (speed < 200 || !IG_RouteAhead(bs, goal, target)) return;
		/* running the way the route goes */
		VectorSubtract(target, bs->origin, d);
		d[2] = 0;
		VectorNormalize(d);
		if (DotProduct(d, bs->cur_ps.velocity) < 0.8f * speed) return;
		EA_Jump(c);
		s->state = IG_SLING_JUMPED;
		s->time = FloatTime();
		s->chain = 0;
		return;

	case IG_SLING_JUMPED:
		/* near the top of the jump, from up there, pick the anchor */
		if (FloatTime() - s->time < IG_JUMP_AIM) return;
		if (!IG_RouteAhead(bs, goal, target) || !IG_FindAnchor(bs, target, s->anchor)) {
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
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
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
				(speed > IG_RELEASE_SPEED && dist > s->remain / 4) ? "fast" :
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
			IG_SlingEnd(s);
			return;
		}
		if (IG_OnGround(bs)) {
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
