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
 * Left out, on purpose: swimming and ladders (the planner treats water as a
 * place not to go; igpm_t.contents says when the box is in it), ducking,
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
float LibVarValue(const char *var_name, const char *value);

#define IGPM_STEPSIZE		18
#define IGPM_MIN_STEP_NORMAL	0.7f
#define IGPM_MAX_CLIP_PLANES	5
#define IGPM_STOP_EPSILON	0.1f

static vec3_t igpm_mins = {-16, -16, -24}, igpm_maxs = {16, 16, 32};
static int igpm_traces;

static void IGPM_Trace(const igpm_t *p, const vec3_t start, const vec3_t end, bsp_trace_t *tr)
{
	vec3_t s, e;

	VectorCopy(start, s);
	VectorCopy(end, e);
	BotAI_Trace(tr, s, igpm_mins, igpm_maxs, e, p->passent, p->mask);
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
}

/* One pmove of `msec`. fmove/smove/upmove as in a usercmd; pitch and yaw the
 * view (Q2 moves along a forward vector with a third of the pitch). */
void IG_Pmove(igpm_t *p, float fmove, float smove, float upmove, float pitch, float yaw, int msec)
{
	float frametime = msec * 0.001f, speed, newspeed, control, wishspeed, add, acc, accel;
	vec3_t angles, fwd, right, wishvel, wishdir;
	int i;

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
	if (wishspeed > p->maxspeed) wishspeed = p->maxspeed;

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

	IGPM_Categorize(p);
	p->time += frametime;
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
}

int IG_PmoveTraces(void)
{
	int n = igpm_traces;

	igpm_traces = 0;
	return n;
}
