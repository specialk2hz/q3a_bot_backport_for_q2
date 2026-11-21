/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_roam.c -- goals with a reason, in place of the random one (IG-7).
 *
 * With nothing to pick up, BotChooseLTGItem (botlib/be_ai_goal.c) falls back
 * to AAS_RandomGoalArea: a random area anywhere on the map, kept for up to 20
 * seconds. In instagib, where every item is removed, that is all the bots
 * ever do between fights. Only THAT goal is replaced here -- an item goal is
 * never touched, so deathmatch's item gathering stays as it is.
 *
 * Hunt: each bot remembers, per enemy, where it last saw or heard it --
 * seen in a 120-degree view, or heard firing within 1500 units -- and goes
 * there. Nothing it has not perceived: it hunts the last trace, not the
 * player. A revenge target (ai_ig_revenge.c) comes first. The goal is passed
 * through, not stood on: within 160 units the bot picks its next one.
 *
 * libvars (set by the game, src/ig/ig_bot.c):
 *   ig_roam          1: replace the random goal (default 0)
 *   ig_hunt_memory   seconds a trace is worth hunting (default 10)
 *   ig_debug         1: the bot says in chat what goal it chose and why
 * High ground and patrol goals are the next steps; without a fresh trace the
 * random goal stays for now.
 */

#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"

#define IG_SEE_FOV      120
#define IG_SEE_DIST     3000
#define IG_HEAR_DIST    1500
#define IG_PASS_DIST    160     /* this close to a hunt goal, choose the next */
#define IG_HUNT_LTG     10      /* seconds a hunt goal is kept at most */
#define IG_REPLAN_NEWER 3       /* a trace this much fresher re-plans the hunt */

typedef struct {
	vec3_t pos;
	float  time;                /* 0: never */
	int    kind;                /* IG_SEEN, IG_HEARD, IG_KILLEDBY */
} ig_trace_t;

static ig_trace_t ig_traces[MAX_CLIENTS][MAX_CLIENTS];   /* [bot][enemy] */
static int        ig_hunting[MAX_CLIENTS];               /* 1 + enemy of the bot's hunt goal, 0: none */
static vec3_t     ig_huntpos[MAX_CLIENTS];
static float      ig_hunttime[MAX_CLIENTS];               /* time of the trace it hunts */

static int IG_RoamOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_roam") != 0;
}

static float IG_HuntMemory(void)
{
	int m = trap_Cvar_VariableIntegerValue("ig_hunt_memory");
	return m > 0 ? m : 10;
}

void IG_EvidenceKind(int bot, int enemy, const vec3_t pos, int kind)
{
	if (bot < 0 || bot >= MAX_CLIENTS || enemy < 0 || enemy >= MAX_CLIENTS) return;
	VectorCopy(pos, ig_traces[bot][enemy].pos);
	ig_traces[bot][enemy].time = FloatTime();
	ig_traces[bot][enemy].kind = kind;
}

void IG_Evidence(int bot, int enemy, const vec3_t pos)
{
	IG_EvidenceKind(bot, enemy, pos, IG_KILLEDBY);
}

/* ig_debug: say it, so a person in the game can judge the choice. */
void IG_Debug(bot_state_t *bs, const char *fmt, ...)
{
	va_list ap;
	char buf[256];

	if (!trap_Cvar_VariableIntegerValue("ig_debug")) return;
	va_start(ap, fmt);
	Q_vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	trap_EA_Say(bs->client, buf);
}

int IG_LastEvidence(int bot, int enemy, vec3_t pos, float *time)
{
	if (bot < 0 || bot >= MAX_CLIENTS || enemy < 0 || enemy >= MAX_CLIENTS) return 0;
	if (ig_traces[bot][enemy].time <= 0 || ig_traces[bot][enemy].time > FloatTime()) return 0;
	VectorCopy(ig_traces[bot][enemy].pos, pos);
	*time = ig_traces[bot][enemy].time;
	return 1;
}

/* Every think, from BotDeathmatchAI: what the bot perceives, and when to
 * move on from a hunt goal. */
void IG_RoamThink(bot_state_t *bs)
{
	int i, c = bs->client;
	float dist;
	vec3_t dir;
	aas_entityinfo_t entinfo;

	if (c < 0 || c >= MAX_CLIENTS) return;
	if (!IG_RoamOn() && IG_RevengeTarget(bs) < 0 &&
		!trap_Cvar_VariableIntegerValue("ig_revenge")) return;
	/* a new map: the traces are of the old one */
	if (ig_hunttime[c] > FloatTime()) {
		memset(ig_traces[c], 0, sizeof(ig_traces[c]));
		ig_hunting[c] = 0;
		ig_hunttime[c] = 0;
	}
	for (i = 0; i < maxclients && i < MAX_CLIENTS; i++) {
		if (i == c) continue;
		BotEntityInfo(i, &entinfo);
		if (!entinfo.valid || EntityIsDead(&entinfo)) continue;
		if (BotSameTeam(bs, i)) continue;
		VectorSubtract(entinfo.origin, bs->origin, dir);
		dist = VectorLength(dir);
		if (EntityIsShooting(&entinfo) && dist < IG_HEAR_DIST)
			IG_EvidenceKind(c, i, entinfo.origin, IG_HEARD);
		else if (dist < IG_SEE_DIST &&
				 BotEntityVisible(bs->entitynum, bs->eye, bs->viewangles, IG_SEE_FOV, i) > 0)
			IG_EvidenceKind(c, i, entinfo.origin, IG_SEEN);
	}
	if (!ig_hunting[c]) return;
	/* pass through: choose the next goal before arriving */
	VectorSubtract(ig_huntpos[c], bs->origin, dir);
	if (VectorLength(dir) < IG_PASS_DIST) {
		ig_hunting[c] = 0;
		bs->ltg_time = 0;
		return;
	}
	/* a fresher trace of anyone re-plans the hunt */
	for (i = 0; i < maxclients && i < MAX_CLIENTS; i++) {
		if (ig_traces[c][i].time > ig_hunttime[c] + IG_REPLAN_NEWER) {
			ig_hunting[c] = 0;
			bs->ltg_time = 0;
			return;
		}
	}
}

/* The trace to hunt: the revenge target's if there is one, else the
 * freshest, if fresh enough and not where the bot already is. */
static int IG_HuntTrace(bot_state_t *bs, vec3_t pos, float *time)
{
	int i, best = -1, nemesis;
	float t, besttime = 0, memory = IG_HuntMemory();
	vec3_t p, dir;

	nemesis = IG_RevengeTarget(bs);
	if (nemesis >= 0 && IG_LastEvidence(bs->client, nemesis, p, &t)) {
		VectorSubtract(p, bs->origin, dir);
		if (VectorLength(dir) > IG_PASS_DIST) {
			VectorCopy(p, pos);
			*time = t;
			return nemesis;
		}
	}
	for (i = 0; i < maxclients && i < MAX_CLIENTS; i++) {
		if (i == bs->client) continue;
		if (!IG_LastEvidence(bs->client, i, p, &t)) continue;
		if (FloatTime() - t > memory || t <= besttime) continue;
		VectorSubtract(p, bs->origin, dir);
		if (VectorLength(dir) < IG_PASS_DIST) continue;
		best = i;
		besttime = t;
		VectorCopy(p, pos);
	}
	*time = besttime;
	return best;
}

/* From BotGetItemLongTermGoal (ai_dmnet.c), right after BotChooseLTGItem put
 * a new goal on the stack: replace it if it is the random fallback. */
void IG_RoamReplace(bot_state_t *bs, int tfl)
{
	bot_goal_t goal;
	vec3_t pos;
	float time;
	int enemy, area, c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS) return;
	ig_hunting[c] = 0;
	if (!IG_RoamOn() && IG_RevengeTarget(bs) < 0) return;
	if (!trap_BotGetTopGoal(bs->gs, &goal)) return;
	/* only AAS_RandomGoalArea's goal: GFL_ROAM and no item (an
	 * item_botroam goal is GFL_ROAM with an item number) */
	if (!(goal.flags & GFL_ROAM) || goal.number != 0) return;
	enemy = IG_HuntTrace(bs, pos, &time);
	if (enemy < 0) {
		IG_Debug(bs, "nothing fresh to hunt, wandering");
		return;
	}
	area = BotPointAreaNum(pos);
	if (!area) return;
	{
		static const char *how[] = { "", "saw", "heard", "was killed by" };
		char name[64];
		int k = ig_traces[c][enemy].kind;

		IG_Debug(bs, "%shunting %s: %s them %.0f s ago",
				 IG_RevengeTarget(bs) == enemy ? "REVENGE: " : "",
				 ClientName(enemy, name, sizeof(name)), how[k >= 1 && k <= 3 ? k : 0],
				 FloatTime() - time);
	}
	trap_BotPopGoal(bs->gs);
	memset(&goal, 0, sizeof(goal));
	VectorCopy(pos, goal.origin);
	VectorSet(goal.mins, -16, -16, -16);
	VectorSet(goal.maxs, 16, 16, 16);
	goal.areanum = area;
	goal.flags = GFL_ROAM;
	trap_BotPushGoal(bs->gs, &goal);
	bs->ltg_time = FloatTime() + IG_HUNT_LTG;
	ig_hunting[c] = 1 + enemy;
	VectorCopy(pos, ig_huntpos[c]);
	ig_hunttime[c] = time;
}
