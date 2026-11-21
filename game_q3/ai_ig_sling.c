/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_sling.c -- slingshot: jump, hook ahead, let go, keep the speed (IG-9).
 *
 * A movement technique on top of the route, not a route: while a bot runs a
 * route on the ground, it looks IG_LOOKAHEAD units along it; when that
 * stretch is long and clear and there is solid wall or ceiling ahead at a
 * shallow angle, it jumps, hooks it in the air, rides the pull for part of
 * the way and lets go, flying on with the speed the pull gave it. The Q3
 * grapple routes (IG-6) only start standing; this is the bots' first
 * mid-air hook.
 *
 * libvars (src/ig/ig_bot.c): ig_sling 1 turns it on (default 0); it needs
 * the offhand hook wiring of IG-6 (ig_hookdetect).
 * Lines "ig: slingshot <bot> ..." are what igbench counts.
 */

#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"

/* botlib/be_ig_hook.c: the hook watchdog, and the bot's own-hook test */
void IG_GrappleTravelNote(int client);
int  IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void EA_Jump(int client);

#define IG_LOOKAHEAD     700    /* how far along the route it looks */
#define IG_MIN_RUN       450    /* the route must run on this far, horizontally */
#define IG_ANCHOR_MIN    350
#define IG_ANCHOR_MAX    1100   /* the hook flies at 1650: at most 0.7 s */
#define IG_ANCHOR_UPMIN  40     /* anchor height above the eye */
#define IG_ANCHOR_UPMAX  450
#define IG_RELEASE_FRAC  0.55f  /* let go this far along the pull ... */
#define IG_RELEASE_SPEED 850    /* ... or this fast and past a third of it */
#define IG_FLY_MAX       0.8f   /* seconds for the hook to latch on */
#define IG_HOOK_MAX      0.8f   /* seconds pulled at most, from the latch */
#define IG_STALL_SPEED   150    /* pulled but this slow: the body is caught */
#define IG_LATCH_RADIUS  220    /* how far from the aimed anchor the hook may latch */
#define IG_FIRE_TIME     0.6f   /* seconds after the jump to get the hook out */
#define IG_AIM_TOL       8      /* degrees */
#define IG_COOLDOWN      3.0f

enum { IG_SLING_IDLE, IG_SLING_JUMPED, IG_SLING_HOOKED };

typedef struct {
	int    state;
	float  time;        /* when the state began */
	float  cooldown;    /* no new slingshot before */
	vec3_t anchor, start;
	float  length;      /* start to anchor */
	float  latched;     /* when the hook latched on, 0: still flying */
	vec3_t latchpos;    /* where the bot was then */
	float  remain;      /* eye to anchor then: the pull's length */
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

/* Is there an anchor for a slingshot toward `target`? */
static int IG_FindAnchor(bot_state_t *bs, const vec3_t target, vec3_t anchor)
{
	static const float pitches[] = { 10, 18, 26 };
	vec3_t dir, flat, end, mid, down, angles;
	vec3_t mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;
	float d, up;
	int i;

	VectorSubtract(target, bs->eye, flat);
	flat[2] = 0;
	if (VectorNormalize(flat) < IG_MIN_RUN) return 0;
	vectoangles(flat, angles);
	for (i = 0; i < 3; i++) {
		angles[PITCH] = -pitches[i];
		AngleVectors(angles, dir, NULL, NULL);
		VectorMA(bs->eye, IG_ANCHOR_MAX, dir, end);
		BotAI_Trace(&tr, bs->eye, NULL, NULL, end, bs->entitynum, MASK_SOLID);
		if (tr.fraction >= 1 || tr.startsolid) continue;
		if (tr.surface.flags & SURF_SKY) continue;     /* the bots do not hook the sky (yet) */
		d = tr.fraction * IG_ANCHOR_MAX;
		up = tr.endpos[2] - bs->eye[2];
		if (d < IG_ANCHOR_MIN || up < IG_ANCHOR_UPMIN || up > IG_ANCHOR_UPMAX) continue;
		/* the body's path to the release point must be clear */
		VectorMA(bs->origin, IG_RELEASE_FRAC * d, dir, mid);
		BotAI_Trace(&tr, bs->origin, mins, maxs, mid, bs->entitynum, MASK_SOLID);
		if (tr.fraction < 1) continue;
		/* and there must be floor (not a pit, not lava) under where it flies on */
		VectorMA(bs->origin, 0.8f * d, dir, mid);
		VectorCopy(mid, down);
		down[2] -= 700;
		BotAI_Trace(&tr, mid, NULL, NULL, down, bs->entitynum, MASK_SOLID | CONTENTS_LAVA | CONTENTS_SLIME);
		if (tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME))) continue;
		VectorMA(bs->eye, d, dir, anchor);
		return 1;
	}
	return 0;
}

/* From AINode_Seek_LTG (ai_dmnet.c), after the route's movement and view. */
void IG_Slingshot(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult)
{
	ig_sling_t *s;
	vec3_t target, d;
	float speed, dist;
	int c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS || !IG_SlingOn()) return;
	s = &ig_sling[c];
	/* a new map: the clock went back */
	if (s->time > FloatTime() + 1 || s->cooldown > FloatTime() + IG_COOLDOWN + 5) memset(s, 0, sizeof(*s));

	switch (s->state) {
	case IG_SLING_IDLE:
		if (FloatTime() < s->cooldown) return;
		if (bs->cur_ps.groundEntityNum == ENTITYNUM_NONE) return;   /* starts from the ground */
		if ((moveresult->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) return;
		if (moveresult->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)) return;
		speed = VectorLength(bs->cur_ps.velocity);
		if (speed < 200) return;
		if (!trap_BotMovementViewTarget(bs->ms, goal, bs->tfl, IG_LOOKAHEAD, target)) return;
		/* running the way the route goes */
		VectorSubtract(target, bs->origin, d);
		d[2] = 0;
		VectorNormalize(d);
		if (DotProduct(d, bs->cur_ps.velocity) < 0.8f * speed) return;
		if (!IG_FindAnchor(bs, target, s->anchor)) {
			s->cooldown = FloatTime() + 0.5f;   /* look again in a moment */
			return;
		}
		VectorCopy(bs->origin, s->start);
		VectorSubtract(s->anchor, bs->eye, d);
		s->length = VectorLength(d);
		EA_Jump(c);
		IG_ViewAt(bs, s->anchor);
		s->state = IG_SLING_JUMPED;
		s->time = FloatTime();
		return;

	case IG_SLING_JUMPED:
		IG_ViewAt(bs, s->anchor);
		if (FloatTime() - s->time > IG_FIRE_TIME) {     /* never got aimed: forget it */
			s->state = IG_SLING_IDLE;
			s->cooldown = FloatTime() + IG_COOLDOWN;
			return;
		}
		if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE && FloatTime() - s->time > 0.15f) return;
		if (!IG_Aimed(bs)) return;
		trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
		IG_GrappleTravelNote(c);
		BotAI_Print(PRT_MESSAGE, "ig: slingshot %d length %.0f\n", c, s->length);
		IG_Debug(bs, IG_DBG_RARE, "slingshot: %.0f units", s->length);
		s->state = IG_SLING_HOOKED;
		s->time = FloatTime();
		s->latched = 0;
		return;

	case IG_SLING_HOOKED:
		IG_ViewAt(bs, s->anchor);
		IG_GrappleTravelNote(c);        /* ours: the watchdog keeps off */
		speed = VectorLength(bs->cur_ps.velocity);
		/* the flight: the pull only counts from the latch -- the bot runs on
		 * while the hook flies, so measured from the start it was already
		 * past the release point when the pull began */
		if (!s->latched) {
			vec3_t hookpos;

			if (IG_HookNear(s->anchor, IG_LATCH_RADIUS, hookpos) == 2) {
				/* from here on the anchor is where it actually holds */
				VectorCopy(hookpos, s->anchor);
				s->latched = FloatTime();
				VectorCopy(bs->origin, s->latchpos);
				VectorSubtract(s->anchor, bs->eye, d);
				s->remain = VectorLength(d);
			}
			else if (FloatTime() - s->time < IG_FLY_MAX) return;
		}
		VectorSubtract(bs->origin, s->latched ? s->latchpos : s->start, d);
		dist = VectorLength(d);
		{
		const char *why = !s->latched ? "nolatch" :
			dist > IG_RELEASE_FRAC * s->remain ? "far" :
			(speed > IG_RELEASE_SPEED && dist > s->remain / 4) ? "fast" :
			FloatTime() - s->latched > IG_HOOK_MAX ? "time" :
			(FloatTime() - s->latched > 0.15f && speed < IG_STALL_SPEED) ? "stall" :
			!IG_HookNear(s->anchor, 48, d) ? "lost" : NULL;
		if (why) {
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			BotAI_Print(PRT_MESSAGE, "ig: slingshot release %d speed %.0f after %.0f why %s pulled %.1f\n", c, speed, dist, why,
						s->latched ? FloatTime() - s->latched : 0);
			s->state = IG_SLING_IDLE;
			s->cooldown = FloatTime() + IG_COOLDOWN;
		}
		}
		return;
	}
}
