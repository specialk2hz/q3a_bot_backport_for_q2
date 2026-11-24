/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_pred.c -- movement planner, step 1: how well does the movement
 * predictor (be_aas_move.c, AAS_PredictClientMovement) match a bot's real
 * Quake II movement?
 *
 * libvar ig_predcheck 1 (default 0; testing aid, never in play). Every think,
 * each bot's state (origin, velocity, on ground, own hook) and the input it
 * sends this frame are kept. Then, from the state 1, 5 and 10 thinks back, the
 * predictor is replayed with the inputs the bot really sent, one think at a
 * time, and compared with where the bot is now. Two physics: 0 = stock Q3
 * (one 0.1 s step per think), 1 = aas_ig_q2phys (two 0.05 s steps per think,
 * as the game runs a bot's pmove). Plus 9 = no physics at all, position +
 * velocity * time, the baseline a predictor has to beat. Windows with the
 * bot's own hook out, a death, or a jump in position (teleport, respawn,
 * rescue) are left out. One line per check:
 *   igpred <client> <k> <mode> <horizontal error> <vertical error> <start: 0 air 1 ground> <jumps> <landings>
 */
#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"
#include "../botlib/be_ig_hook.h"
#include <time.h>

int AAS_OnGround(vec3_t origin, int presencetype, int passent);
extern int aas_ig_q2phys;

#define IGP_HIST	13

typedef struct {
	vec3_t	org, vel, cmd;
	float	fm, sm, pitch, yaw;
	int		ground, hook, jump, dead;	/* jump: 1 both halves, 2 the second half (delayed) */
} igp_frame_t;

static igp_frame_t	igp[MAX_CLIENTS][IGP_HIST];
static int			igp_n[MAX_CLIENTS];		/* frames stored, total */

/* The input as the game turns it into a usercmd (bl_main.c BotInputToUser-
 * Command's Q2 twin): forward and side from the view's yaw, the key flags
 * adding 400; pmove caps the wish speed at 300 itself. */
static void IGP_Cmd(const bot_input_t *bi, igp_frame_t *f)
{
	vec3_t ang, fwd, right;
	float fm, sm;

	VectorSet(ang, 0, bi->viewangles[YAW], 0);
	AngleVectors(ang, fwd, right, NULL);
	fm = DotProduct(fwd, bi->dir) * bi->speed;
	sm = DotProduct(right, bi->dir) * bi->speed;
	if (bi->actionflags & ACTION_MOVEFORWARD) fm += 400;
	if (bi->actionflags & ACTION_MOVEBACK) fm -= 400;
	if (bi->actionflags & ACTION_MOVELEFT) sm -= 400;
	if (bi->actionflags & ACTION_MOVERIGHT) sm += 400;
	f->fm = fm > 400 ? 400 : fm < -400 ? -400 : fm;
	f->sm = sm > 400 ? 400 : sm < -400 ? -400 : sm;
	f->pitch = bi->viewangles[PITCH];
	f->yaw = bi->viewangles[YAW];
	f->cmd[0] = fwd[0] * fm + right[0] * sm;
	f->cmd[1] = fwd[1] * fm + right[1] * sm;
	f->cmd[2] = 0;
	f->jump = (bi->actionflags & ACTION_JUMP) ? 1 : (bi->actionflags & ACTION_DELAYEDJUMP) ? 2 : 0;
	if (f->jump) f->cmd[2] = 400;
	else if (bi->actionflags & ACTION_CROUCH) f->cmd[2] = -400;
}

/* Replay k thinks from frame `from` (oldest first) with one physics mode. */
static void IGP_Replay(bot_state_t *bs, int from, int k, int mode, int shift, vec3_t out)
{
	vec3_t org, vel, cmd;
	int i, ground, steps, prevjump;
	float ft;
	aas_clientmove_t move;
	igp_frame_t *f = &igp[bs->client][from % IGP_HIST];

	VectorCopy(f->org, org);
	VectorCopy(f->vel, vel);
	ground = f->ground;
	if (mode == 9) {
		VectorMA(org, 0.1f * k, vel, out);
		return;
	}
	if (mode == 2) {	/* the Q2 pmove port: two 50 ms halves per think, as bl_main.c runs them */
		igpm_t pm;

		IG_PmoveInit(&pm, org, vel, ground, bs->entitynum, 1);
		pm.jumpheld = from > 0 && igp[bs->client][(from - 1) % IGP_HIST].jump;
		for (i = 0; i < k; i++) {
			igp_frame_t *c = &igp[bs->client][(from + i) % IGP_HIST];

			IG_Pmove(&pm, c->fm, c->sm, c->jump == 1 ? 400 : 0, c->pitch, c->yaw, 50);
			IG_Pmove(&pm, c->fm, c->sm, c->jump ? 400 : 0, c->pitch, c->yaw, 50);
		}
		VectorCopy(pm.org, out);
		return;
	}
	steps = mode == 1 ? 2 : 1;
	ft = 0.1f / steps;
	prevjump = from > 0 ? igp[bs->client][(from - 1) % IGP_HIST].jump : 0;
	aas_ig_q2phys = mode == 1;
	for (i = 0; i < k; i++) {
		int ci = from + i + shift;
		igp_frame_t *c;

		if (ci < 0) ci = 0;
		if (ci > igp_n[bs->client] - 1) ci = igp_n[bs->client] - 1;
		c = &igp[bs->client][ci % IGP_HIST];
		int s;

		VectorCopy(c->cmd, cmd);
		/* Q2 jumps only on a fresh press (PMF_JUMP_HELD) */
		if (mode == 1 && c->jump && prevjump) cmd[2] = 0;
		prevjump = c->jump;
		for (s = 0; s < steps; s++) {
			Com_Memset(&move, 0, sizeof(move));
			trap_AAS_PredictClientMovement(&move, bs->entitynum, org, PRESENCE_NORMAL, ground,
										   vel, cmd, 1, 1, ft, 0, 0, false);
			VectorCopy(move.endpos, org);
			org[2] -= 0.25f;	/* the predictor lifts the start by 0.25 every call */
			VectorCopy(move.velocity, vel);
			ground = AAS_OnGround(org, PRESENCE_NORMAL, bs->entitynum);
			if (cmd[2] > 0) cmd[2] = 0;	/* a jump is one frame's */
		}
	}
	aas_ig_q2phys = 0;
	VectorCopy(org, out);
}

void IG_PredCheck(bot_state_t *bs, const bot_input_t *bi)
{
	static const int ks[3] = {1, 5, 10};
	static const int modes[4] = {0, 1, 2, 9}, shifts[4] = {0, 0, 0, 0};
	static int checks, usec;
	int c = bs->client, now, i, m, j;
	igp_frame_t *f;
	vec3_t viewoffset, anchor;

	if (!trap_Cvar_VariableIntegerValue("ig_predcheck") || c < 0 || c >= MAX_CLIENTS)
		return;
	now = igp_n[c];
	f = &igp[c][now % IGP_HIST];
	VectorCopy(bs->cur_ps.origin, f->org);
	VectorCopy(bs->cur_ps.velocity, f->vel);
	f->ground = bs->cur_ps.groundEntityNum != ENTITYNUM_NONE;
	VectorSet(viewoffset, 0, 0, bs->cur_ps.viewheight);
	f->hook = IG_GrappleState(bs->origin, viewoffset, anchor) != 0;
	f->dead = BotIsDead(bs);
	IGP_Cmd(bi, f);
	igp_n[c]++;
	if (trap_Cvar_VariableIntegerValue("ig_predcheck") == 2) {
		vec3_t ang, fwd, right;

		VectorSet(ang, 0, bi->viewangles[YAW], 0);
		AngleVectors(ang, fwd, right, NULL);
		BotAI_Print(PRT_MESSAGE, "igpcmd %d %.0f %.0f %d %.1f x %.0f y %.0f vx %.0f vy %.0f g %d\n", c,
					DotProduct(fwd, f->cmd), DotProduct(right, f->cmd), f->jump, bi->viewangles[YAW],
					f->org[0], f->org[1], f->vel[0], f->vel[1], f->ground);
		return;
	}

	for (i = 0; i < 3; i++) {
		int k = ks[i], from = now - k, ok = 1, jumps = 0, lands = 0;
		vec3_t pred;

		if (from < 0) continue;
		for (j = from; j <= now && ok; j++) {
			igp_frame_t *a = &igp[c][j % IGP_HIST];

			if (a->hook || a->dead) ok = 0;
			if (j > from) {
				igp_frame_t *b = &igp[c][(j - 1) % IGP_HIST];
				vec3_t mid, d;

				/* moved further than its velocities allow: teleported */
				VectorAdd(a->vel, b->vel, mid);
				VectorScale(mid, 0.05f, mid);
				VectorSubtract(a->org, b->org, d);
				VectorSubtract(d, mid, d);
				if (VectorLength(d) > 48) ok = 0;
				if (!b->ground && a->ground) lands++;
			}
			if (j < now && a->jump) jumps++;
		}
		if (!ok) continue;
		for (m = 0; m < 4; m++) {
			float dh;

			struct timespec t0, t1;

			clock_gettime(CLOCK_MONOTONIC, &t0);
			IGP_Replay(bs, from, k, modes[m], shifts[m], pred);
			clock_gettime(CLOCK_MONOTONIC, &t1);
			if (modes[m] == 2) {
				usec += (t1.tv_sec - t0.tv_sec) * 1000000 + (t1.tv_nsec - t0.tv_nsec) / 1000;
				checks += 2 * k;
				if (checks >= 20000) {
					BotAI_Print(PRT_MESSAGE, "igpmtime %d steps %.2f us/step %.1f traces/step\n", checks,
								(float)usec / checks, (float)IG_PmoveTraces() / checks);
					checks = usec = 0;
				}
			}
			dh = sqrt((pred[0] - f->org[0]) * (pred[0] - f->org[0]) + (pred[1] - f->org[1]) * (pred[1] - f->org[1]));
			BotAI_Print(PRT_MESSAGE, "igpred %d %d %d %.1f %.1f %d %d %d\n", c, k, modes[m], dh,
						pred[2] - f->org[2], igp[c][from % IGP_HIST].ground, jumps, lands);
		}
	}
}
