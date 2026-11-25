/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_pmove.c -- Quake II's player movement, for the bots to predict their
 * own flights (movement planner, IG-40).
 *
 * Ported from Q2PRO's src/common/pmove/template.c (id's pmove.c, GPL), the
 * code the server runs for every player, bots included: friction, ground and
 * air acceleration, gravity, PM_StepSlideMove (slide along walls, step up
 * 18-unit steps), PM_CategorizePosition (on ground: a 0.7 floor within 0.25
 * units) and PM_CheckJump (a fresh press, 270 up). Collision is the real map
 * through BotAI_Trace, not the navigation mesh.
 *
 * The hook (step 2) as osp_hook.c and ig_hook.c run it: fired from the hand
 * (P_ProjectSource 24 8 viewheight-8) along the view, flying at ig_hookfly,
 * latching on the first solid it touches (the sky only with ig_hooksky);
 * after each pmove the pull SETS the velocity toward the anchor from the eye:
 * OSP at ig_hookpull, 425 once inside 64 (hang); LMCTF (ig_hookstyle 1) at
 * ig_hookpull minus a frame of gravity, easing in inside 120 units.
 *
 * Ducking as PM_CheckDuck: a crouch on the ground ducks (box top 4, run speed
 * 100), standing again when there is room.
 *
 * Jump pads and teleporters are flagged when the box enters their navigation
 * area (igpm_t.pad, .teleport), not simulated.
 *
 * Left out, on purpose: swimming and ladders (the planner treats water as a
 * place not to go; igpm_t.contents says when the box is in it),
 * conveyor currents, the 1/8-unit snapping, and the landing jump lock (bots
 * play with ig_botstrafehack 1, which clears it).
 *
 * Why not AAS_PredictClientMovement (be_aas_move.c): measured against bots'
 * real movement on q2rdm11 (ai_ig_pred.c, ig_predcheck 1), it was 45 units off
 * after 0.5 s of plain running where Q2's own rules are 1 unit off.
 */
#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"

int AAS_PointContents(vec3_t point);
int AAS_PointAreaNum(vec3_t point);
int AAS_AreaJumpPad(int areanum);
int AAS_AreaTeleporter(int areanum);
int AAS_AreaLava(int areanum);
int AAS_AreaSlime(int areanum);
float LibVarValue(const char *var_name, const char *value);

#define IGPM_STEPSIZE		18
#define IGPM_MIN_STEP_NORMAL	0.7f
#define IGPM_MAX_CLIP_PLANES	5
#define IGPM_STOP_EPSILON	0.1f

static vec3_t igpm_mins = {-16, -16, -24}, igpm_maxs = {16, 16, 32}, igpm_maxsducked = {16, 16, 4};
static int igpm_traces;

static void IGPM_Trace(const igpm_t *p, const vec3_t start, const vec3_t end, bsp_trace_t *tr)
{
	vec3_t s, e;

	VectorCopy(start, s);
	VectorCopy(end, e);
	BotAI_Trace(tr, s, igpm_mins, p->ducked ? igpm_maxsducked : igpm_maxs, e, p->passent, p->mask);
	igpm_traces++;
}

static void IGPM_ClipVelocity(const vec3_t in, const vec3_t normal, vec3_t out, float overbounce)
{
	float backoff = DotProduct(in, normal) * overbounce, change;
	int i;

	for (i = 0; i < 3; i++) {
		change = normal[i] * backoff;
		out[i] = in[i] - change;
		if (out[i] > -IGPM_STOP_EPSILON && out[i] < IGPM_STOP_EPSILON)
			out[i] = 0;
	}
}

static void IGPM_SlideMove(igpm_t *p, float frametime)
{
	vec3_t planes[IGPM_MAX_CLIP_PLANES], primal, end, dir;
	int bump, numplanes = 0, i, j;
	float time_left = frametime, d;
	bsp_trace_t tr;

	VectorCopy(p->vel, primal);
	for (bump = 0; bump < 4; bump++) {
		VectorMA(p->org, time_left, p->vel, end);
		IGPM_Trace(p, p->org, end, &tr);
		if (tr.allsolid) {
			p->vel[2] = 0;
			return;
		}
		if (tr.fraction > 0) {
			VectorCopy(tr.endpos, p->org);
			numplanes = 0;
		}
		if (tr.fraction == 1)
			break;
		p->blocked = 1;
		if (tr.ent >= 0 && tr.ent < MAX_CLIENTS) p->hitplayer = 1;
		time_left -= time_left * tr.fraction;
		if (numplanes >= IGPM_MAX_CLIP_PLANES) {
			VectorClear(p->vel);
			break;
		}
		VectorCopy(tr.plane.normal, planes[numplanes]);
		numplanes++;
		for (i = 0; i < numplanes; i++) {
			IGPM_ClipVelocity(p->vel, planes[i], p->vel, 1.01f);
			for (j = 0; j < numplanes; j++)
				if (j != i && DotProduct(p->vel, planes[j]) < 0)
					break;
			if (j == numplanes)
				break;
		}
		if (i == numplanes) {
			if (numplanes != 2) {
				VectorClear(p->vel);
				break;
			}
			CrossProduct(planes[0], planes[1], dir);
			d = DotProduct(dir, p->vel);
			VectorScale(dir, d, p->vel);
		}
		if (DotProduct(p->vel, primal) <= 0) {
			VectorClear(p->vel);
			break;
		}
	}
}

static void IGPM_StepSlideMove(igpm_t *p, float frametime)
{
	vec3_t start_o, start_v, down_o, down_v, up, down;
	float down_dist, up_dist;
	bsp_trace_t tr;

	VectorCopy(p->org, start_o);
	VectorCopy(p->vel, start_v);
	IGPM_SlideMove(p, frametime);
	VectorCopy(p->org, down_o);
	VectorCopy(p->vel, down_v);

	VectorCopy(start_o, up);
	up[2] += IGPM_STEPSIZE;
	IGPM_Trace(p, up, up, &tr);
	if (tr.allsolid)
		return;
	VectorCopy(up, p->org);
	VectorCopy(start_v, p->vel);
	IGPM_SlideMove(p, frametime);

	VectorCopy(p->org, down);
	down[2] -= IGPM_STEPSIZE;
	IGPM_Trace(p, p->org, down, &tr);
	if (!tr.allsolid)
		VectorCopy(tr.endpos, p->org);
	VectorCopy(p->org, up);

	down_dist = (down_o[0] - start_o[0]) * (down_o[0] - start_o[0]) + (down_o[1] - start_o[1]) * (down_o[1] - start_o[1]);
	up_dist = (up[0] - start_o[0]) * (up[0] - start_o[0]) + (up[1] - start_o[1]) * (up[1] - start_o[1]);
	if (down_dist > up_dist || tr.plane.normal[2] < IGPM_MIN_STEP_NORMAL) {
		VectorCopy(down_o, p->org);
		VectorCopy(down_v, p->vel);
		return;
	}
	p->vel[2] = down_v[2];
}

static void IGPM_Categorize(igpm_t *p)
{
	vec3_t point, feet;
	bsp_trace_t tr;

	VectorCopy(p->org, point);
	point[2] -= 0.25f;
	if (p->vel[2] > 180) {
		p->ground = 0;
	} else {
		IGPM_Trace(p, p->org, point, &tr);
		p->ground = (tr.fraction < 1 || tr.startsolid) && !(tr.plane.normal[2] < 0.7f && !tr.startsolid);
		if (p->ground) VectorCopy(tr.plane.normal, p->groundnormal);
	}
	VectorCopy(p->org, feet);
	feet[2] += igpm_mins[2] + 1;
	p->contents = AAS_PointContents(feet);
	p->everhit |= p->contents;
	/* jump pads and teleporters are not simulated: the run says it reached
	 * one, and the planner takes the pad's landing from the navigation file */
	p->area = AAS_PointAreaNum(p->org);
	if (p->area) {
		if (AAS_AreaJumpPad(p->area)) p->pad = 1;
		if (AAS_AreaTeleporter(p->area)) p->teleport = 1;
		/* bspc makes a trigger_hurt (a pit's death zone) lava in the
		 * navigation file, where the map itself has none: a dodge into
		 * tltf2's and brutalo's voids was "safe" while there was a floor
		 * somewhere below. Lava in the navigation file is lava here. */
		if (AAS_AreaLava(p->area)) p->everhit |= CONTENTS_LAVA;
		else if (AAS_AreaSlime(p->area)) p->everhit |= CONTENTS_SLIME;
	}
}

#define IGPM_VIEWHEIGHT	22

static void IGPM_HookPull(igpm_t *p);

/* The order the game runs things in: a bot's two 50 ms ClientThinks (each a
 * pmove, then GrapplePull) come before the frame's entities move, so a hook
 * fired at a think flies at that frame's end, 0.1 s of flight at once, and a
 * hook that bites there pulls from the next frame's first ClientThink, after
 * its pmove. */
static void IGPM_HookFrame(igpm_t *p, float frametime)
{
	vec3_t end;
	bsp_trace_t tr;

	if (p->hook >= 2) {
		IGPM_HookPull(p);
		return;
	}
	if (p->hook == 1 && (p->steps & 1)) {	/* flying: G_RunEntity's MOVETYPE_FLYMISSILE, a point */
		frametime = 0.1f;
		VectorMA(p->hookpos, p->hookfly * frametime, p->hookdir, end);
		BotAI_Trace(&tr, p->hookpos, NULL, NULL, end, p->passent, CONTENTS_SOLID | CONTENTS_BODY);
		igpm_traces++;
		p->hookflytime += frametime;
		VectorCopy(tr.endpos, p->hookpos);
		if (tr.fraction < 1) {
			if (((tr.surface.flags & SURF_SKY) && !p->hooksky) || (tr.ent >= 0 && tr.ent < MAX_CLIENTS)) {
				p->hook = 0;	/* the sky, or a player: lost */
				p->hooklost = 1;
				return;
			}
			p->hook = 2;
		} else if (p->hookflytime > 2) {
			p->hook = 0;
			p->hooklost = 1;
		}
	}
}

/* GrapplePull, after the pmove */
static void IGPM_HookPull(igpm_t *p)
{
	vec3_t eye, dir;
	float vlen, speed;

	VectorCopy(p->org, eye);
	eye[2] += p->ducked ? -2 : IGPM_VIEWHEIGHT;
	VectorSubtract(p->hookpos, eye, dir);
	vlen = VectorNormalize(dir);
	if (p->hook == 2 && vlen < 64 && p->hookstyle == 0)
		p->hook = 3;
	if (p->hookstyle == 1) {	/* LMCTF: gravity still sags, glide in */
		static const struct { float above, times; } ease[] = {
			{ 100, 5 }, { 80, 4 }, { 40, 3 }, { 20, 2 }, { 10, 1 }
		};
		int i;

		if (vlen > 120) {
			VectorScale(dir, p->hookpull, p->vel);
			p->vel[2] -= p->gravity * 0.1f;
			return;
		}
		speed = 1;
		for (i = 0; i < (int)(sizeof(ease) / sizeof(ease[0])); i++)
			if (vlen > ease[i].above) { speed = vlen * ease[i].times; break; }
		VectorScale(dir, speed, p->vel);
		return;
	}
	VectorScale(dir, p->hook == 3 ? 425 : (int)p->hookpull, p->vel);
}

void IG_PmoveHookFire(igpm_t *p, float pitch, float yaw)
{
	vec3_t angles, fwd, right;

	VectorSet(angles, pitch, yaw, 0);
	AngleVectors(angles, fwd, right, NULL);
	VectorCopy(p->org, p->hookpos);	/* P_ProjectSource: 24 forward, 8 right, viewheight - 8 up */
	VectorMA(p->hookpos, 24, fwd, p->hookpos);
	VectorMA(p->hookpos, 8, right, p->hookpos);
	p->hookpos[2] += IGPM_VIEWHEIGHT - 8;
	VectorCopy(fwd, p->hookdir);
	p->hook = 1;
	p->hooklost = 0;
	p->hookflytime = 0;
}

void IG_PmoveHookRelease(igpm_t *p)
{
	p->hook = 0;
}

/* A hook seen in the world: `state` 2 still (bitten), 1 flying along `dir`.
 * One that moved this frame and already touches a surface bit at the end of
 * it: the brain sees it move once more before it is still. */
void IG_PmoveHookAt(igpm_t *p, int state, const vec3_t pos, const vec3_t dir)
{
	p->hook = state;
	VectorCopy(pos, p->hookpos);
	if (dir) VectorCopy(dir, p->hookdir);
	if (state == 1 && dir) {
		vec3_t end;
		bsp_trace_t tr;

		VectorMA(pos, 4, dir, end);
		BotAI_Trace(&tr, p->hookpos, NULL, NULL, end, p->passent, CONTENTS_SOLID | CONTENTS_BODY);
		igpm_traces++;
		if (tr.fraction < 1 && !((tr.surface.flags & SURF_SKY) && !p->hooksky))
			p->hook = 2;
	}
}

/* One pmove of `msec`. fmove/smove/upmove as in a usercmd; pitch and yaw the
 * view (Q2 moves along a forward vector with a third of the pitch). */
void IG_Pmove(igpm_t *p, float fmove, float smove, float upmove, float pitch, float yaw, int msec)
{
	float frametime = msec * 0.001f, speed, newspeed, control, wishspeed, add, acc, accel;
	vec3_t angles, fwd, right, wishvel, wishdir;
	int i;

	/* PM_CheckDuck, with the ground from the last pmove (PMF_ON_GROUND) */
	if (upmove < 0 && p->ground) {
		p->ducked = 1;
	} else if (p->ducked) {
		bsp_trace_t tr;

		p->ducked = 0;
		IGPM_Trace(p, p->org, p->org, &tr);
		if (tr.allsolid) p->ducked = 1;
	}

	IGPM_Categorize(p);

	/* PM_CheckJump */
	if (upmove < 10) {
		p->jumpheld = 0;
	} else if (!p->jumpheld && p->ground && !(p->contents & MASK_WATER)) {
		p->jumpheld = 1;
		p->ground = 0;
		p->vel[2] += 270;
		if (p->vel[2] < 270) p->vel[2] = 270;
		p->jumped = 1;
		p->jumps++;
	}

	/* PM_Friction: ground only (water is out of scope) */
	speed = VectorLength(p->vel);
	if (speed < 1) {
		p->vel[0] = p->vel[1] = 0;
	} else if (p->ground) {
		control = speed < 100 ? 100 : speed;
		newspeed = speed - control * p->friction * frametime;
		if (newspeed < 0) newspeed = 0;
		VectorScale(p->vel, newspeed / speed, p->vel);
	}

	/* PM_AirMove */
	VectorSet(angles, (pitch > 180 ? pitch - 360 : pitch) / 3, yaw, 0);
	AngleVectors(angles, fwd, right, NULL);
	for (i = 0; i < 2; i++)
		wishvel[i] = fwd[i] * fmove + right[i] * smove;
	wishvel[2] = 0;
	VectorCopy(wishvel, wishdir);
	wishspeed = VectorNormalize(wishdir);
	if (wishspeed > (p->ducked ? 100 : p->maxspeed)) wishspeed = p->ducked ? 100 : p->maxspeed;

	accel = p->ground ? 10 : 1;
	if (p->ground) p->vel[2] = 0;
	add = wishspeed - DotProduct(p->vel, wishdir);
	if (add > 0) {
		acc = accel * frametime * wishspeed;
		if (acc > add) acc = add;
		VectorMA(p->vel, acc, wishdir, p->vel);
	}
	if (p->ground) {
		p->vel[2] = 0;
		if (p->vel[0] || p->vel[1])
			IGPM_StepSlideMove(p, frametime);
	} else {
		p->vel[2] -= p->gravity * frametime;
		IGPM_StepSlideMove(p, frametime);
	}

	IGPM_HookFrame(p, frametime);
	IGPM_Categorize(p);
	p->time += frametime;
	p->steps++;
}

void IG_PmoveInit(igpm_t *p, const vec3_t org, const vec3_t vel, int ground, int passent, int withplayers)
{
	Com_Memset(p, 0, sizeof(*p));
	VectorCopy(org, p->org);
	VectorCopy(vel, p->vel);
	p->ground = ground;
	p->passent = passent;
	p->mask = CONTENTS_SOLID | CONTENTS_PLAYERCLIP | (withplayers ? CONTENTS_BODY : 0);
	p->gravity = LibVarValue("phys_gravity", "800");
	if (p->gravity <= 0) p->gravity = 800;
	p->friction = LibVarValue("phys_friction", "6");
	if (p->friction <= 0) p->friction = 6;
	p->maxspeed = 300;
	p->hookstyle = trap_Cvar_VariableIntegerValue("ig_hookstyle");
	p->hooksky = trap_Cvar_VariableIntegerValue("ig_hooksky");
	p->hookfly = LibVarValue("ig_hookfly", "1600");
	p->hookpull = LibVarValue("ig_hookpull", "1000");
}

int IG_PmoveTraces(void)
{
	int n = igpm_traces;

	igpm_traces = 0;
	return n;
}
