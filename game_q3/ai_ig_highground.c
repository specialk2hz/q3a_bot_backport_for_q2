/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/

/* ai_ig_highground.c -- idle bots take the high ground (from IG-7, kept in
 * ig_botnav since 2026-11-28; libvar ig_highground, default 0).
 *
 * With nothing to pick up, BotChooseLTGItem (botlib/be_ai_goal.c) falls back
 * to a random area (IG-44 makes that one fair). This replaces only THAT goal
 * with one of the map's high points (botlib/be_ig_hook.c, IG_HighGround): the
 * nearest by route the bot has not passed in IG_HG_FORGET seconds and no
 * other bot is heading for, among the three nearest at random for variety.
 * Passed through, not stood on: within IG_PASS_DIST the bot picks its next
 * goal. Item goals are never touched, so deathmatch's item gathering stays.
 * (IG-7's hunt for enemies seen or heard and IG-8's revenge were removed
 * 2026-11-28; without the high ground, time on upper levels fell 29 -> 23%.)
 */

#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"
float LibVarValue(const char *var_name, const char *value);	/* botlib, as ai_ig_plan.c */

#define IG_PASS_DIST  160     /* this close to the point, choose the next goal */
#define IG_HG_N       16
#define IG_HG_FORGET  30      /* seconds before a point passed is worth going to again */
#define IG_HG_LTG     15      /* seconds the goal is kept at most */

int IG_HighGround(int max, int *areas, vec3_t *points);   /* botlib */

static vec3_t ig_hgpos[MAX_CLIENTS];
static float  ig_hgvisited[MAX_CLIENTS][IG_HG_N];
static int    ig_hgtarget[MAX_CLIENTS];      /* 1 + point index, 0: none */

static int IG_HighGroundOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_highground") != 0;
}

/* Every think, from BotDeathmatchAI: passed the point? */
void IG_HighGroundThink(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t d;

	if (c < 0 || c >= MAX_CLIENTS || !ig_hgtarget[c] || !IG_HighGroundOn()) return;
	VectorSubtract(ig_hgpos[c], bs->origin, d);
	if (VectorLength(d) < IG_PASS_DIST) {
		ig_hgvisited[c][ig_hgtarget[c] - 1] = FloatTime();
		ig_hgtarget[c] = 0;
		bs->ltg_time = 0;
	}
}

/* From BotGetItemLongTermGoal (ai_dmnet.c), right after BotChooseLTGItem put
 * a new goal on the stack: replace it if it is the random fallback. */
void IG_HighGroundReplace(bot_state_t *bs)
{
	int areas[IG_HG_N], n, i, b, c = bs->client, best[3] = {-1, -1, -1}, pick;
	int besttime[3] = {0, 0, 0}, t;
	vec3_t points[IG_HG_N], d;
	bot_goal_t goal;

	if (c < 0 || c >= MAX_CLIENTS) return;
	ig_hgtarget[c] = 0;
	if (!IG_HighGroundOn()) return;
	if (!trap_BotGetTopGoal(bs->gs, &goal)) return;
	/* only the random goal: GFL_ROAM and no item (an item_botroam goal is
	 * GFL_ROAM with an item number) */
	if (!(goal.flags & GFL_ROAM) || goal.number != 0) return;
	/* only some of the time: on every idle goal the bots met less (28 maps:
	 * upper 29 -> 34%, frags -0.26/min); the rest keep the fair random goal
	 * (IG-44). ig_highgroundchance */
	if (random() >= LibVarValue("ig_highgroundchance", "0.5")) return;
	n = IG_HighGround(IG_HG_N, areas, points);
	for (i = 0; i < n; i++) {
		if (ig_hgvisited[c][i] > FloatTime()) ig_hgvisited[c][i] = 0;          /* a new map */
		if (ig_hgvisited[c][i] && FloatTime() - ig_hgvisited[c][i] < IG_HG_FORGET) continue;
		VectorSubtract(points[i], bs->origin, d);
		if (VectorLength(d) < IG_PASS_DIST * 2) continue;
		for (b = 0; b < MAX_CLIENTS; b++)
			if (b != c && ig_hgtarget[b] == i + 1) break;
		if (b < MAX_CLIENTS) continue;
		t = trap_AAS_AreaTravelTimeToGoalArea(bs->areanum, bs->origin, areas[i], bs->tfl);
		if (t <= 0) continue;
		/* keep the three nearest */
		if (best[2] < 0 || t < besttime[2]) {
			best[2] = i; besttime[2] = t;
			if (best[1] < 0 || t < besttime[1]) {
				best[2] = best[1]; besttime[2] = besttime[1]; best[1] = i; besttime[1] = t;
				if (best[0] < 0 || t < besttime[0]) {
					best[1] = best[0]; besttime[1] = besttime[0]; best[0] = i; besttime[0] = t;
				}
			}
		}
	}
	if (best[0] < 0) return;	/* no high ground to take: the random goal stays */
	pick = best[best[1] >= 0 ? (best[2] >= 0 ? rand() % 3 : rand() % 2) : 0];
	trap_BotPopGoal(bs->gs);
	memset(&goal, 0, sizeof(goal));
	VectorCopy(points[pick], goal.origin);
	VectorSet(goal.mins, -16, -16, -16);
	VectorSet(goal.maxs, 16, 16, 16);
	goal.areanum = areas[pick];
	goal.flags = GFL_ROAM;
	trap_BotPushGoal(bs->gs, &goal);
	bs->ltg_time = FloatTime() + IG_HG_LTG;
	ig_hgtarget[c] = pick + 1;
	VectorCopy(points[pick], ig_hgpos[c]);
}
