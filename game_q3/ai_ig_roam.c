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
#include "chars.h"

#define IG_SEE_FOV      120
#define IG_SEE_DIST     3000
#define IG_HEAR_DIST    1500
#define IG_PASS_DIST    160     /* this close to a hunt goal, choose the next */
#define IG_HUNT_LTG     10      /* seconds a hunt goal is kept at most */
#define IG_REPLAN_NEWER 3       /* a trace this much fresher re-plans the hunt */
#define IG_SHARE_DIST   400     /* another bot hunting within this of a trace has it */
#define IG_NEAR_SCALE   800     /* a trace this far away is worth half one at the bot's feet */
#define IG_HEARD_WEIGHT 0.6f    /* a sound is worth less than a sighting */

typedef struct {
	vec3_t pos;
	float  time;                /* 0: never */
	int    kind;                /* IG_SEEN, IG_HEARD, IG_KILLEDBY */
} ig_trace_t;

static ig_trace_t ig_traces[MAX_CLIENTS][MAX_CLIENTS];   /* [bot][enemy] */
static int        ig_hunting[MAX_CLIENTS];               /* 1 + enemy of the bot's hunt goal, 0: none */
static vec3_t     ig_huntpos[MAX_CLIENTS];
static float      ig_hunttime[MAX_CLIENTS];               /* time of the trace it hunts */
static vec3_t     ig_hgpos[MAX_CLIENTS];                  /* the high ground point it heads for */
static void IG_HighGroundGoal(bot_state_t *bs);
#define IG_HG_N       16
#define IG_HG_FORGET  30     /* seconds before a point passed is worth going to again */
#define IG_HG_LTG     15
static float ig_hgvisited[MAX_CLIENTS][IG_HG_N];
static int   ig_hgtarget[MAX_CLIENTS];      /* 1 + point index, 0: none */

static int IG_RoamOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_roam") != 0;
}

/* IG-27 ig_peace: bots pick no enemies (BotFindEnemy) and keep no traces to
 * hunt, so they only move: for watching their movement and testing routes. */
int IG_Peace(void)
{
	return trap_Cvar_VariableIntegerValue("ig_peace") != 0;
}

static float IG_HuntMemory(void)
{
	int m = trap_Cvar_VariableIntegerValue("ig_hunt_memory");
	return m > 0 ? m : 10;
}

void IG_EvidenceKind(int bot, int enemy, const vec3_t pos, int kind)
{
	if (bot < 0 || bot >= MAX_CLIENTS || enemy < 0 || enemy >= MAX_CLIENTS) return;
	if (IG_Peace()) return;
	VectorCopy(pos, ig_traces[bot][enemy].pos);
	ig_traces[bot][enemy].time = FloatTime();
	ig_traces[bot][enemy].kind = kind;
}

void IG_Evidence(int bot, int enemy, const vec3_t pos)
{
	IG_EvidenceKind(bot, enemy, pos, IG_KILLEDBY);
}

/* ig_debug: say it, so a person in the game can judge the choice.
 * IG_DBG_RARE lines (grudges, slingshots) always; IG_DBG_HUNT lines at most
 * one per bot every IG_DBG_GAP seconds; IG_DBG_CHATTY lines only at
 * ig_debug 2, which also lifts the gap. */
#define IG_DBG_GAP 10
static float ig_dbglast[MAX_CLIENTS];

void IG_Debug(bot_state_t *bs, int kind, const char *fmt, ...)
{
	va_list ap;
	char buf[256];
	int level = trap_Cvar_VariableIntegerValue("ig_debug"), c = bs->client;

	if (level <= 0 || c < 0 || c >= MAX_CLIENTS) return;
	if (kind == IG_DBG_CHATTY && level < 2) return;
	if (kind == IG_DBG_HUNT && level < 2) {
		if (ig_dbglast[c] > FloatTime()) ig_dbglast[c] = 0;       /* a new map */
		if (FloatTime() - ig_dbglast[c] < IG_DBG_GAP) return;
		ig_dbglast[c] = FloatTime();
	}
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
	if (ig_hgtarget[c]) {
		VectorSubtract(ig_hgpos[c], bs->origin, dir);
		if (VectorLength(dir) < IG_PASS_DIST) {
			ig_hgvisited[c][ig_hgtarget[c] - 1] = FloatTime();
			ig_hgtarget[c] = 0;
			bs->ltg_time = 0;
			return;
		}
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

/* Is another bot already hunting this enemy, or near this spot? Without the
 * check every bot that heard the same rail shot went to it, and the fight it
 * found drew them all in again: crowded 15% -> 23% of the time. */
static int IG_TraceTaken(int c, int enemy, const vec3_t p)
{
	int b;
	vec3_t d;

	for (b = 0; b < MAX_CLIENTS; b++) {
		if (b == c || !ig_hunting[b]) continue;
		if (ig_hunting[b] == enemy + 1) return 1;
		VectorSubtract(ig_huntpos[b], p, d);
		if (VectorLength(d) < IG_SHARE_DIST) return 1;
	}
	return 0;
}

/* The trace to hunt: the revenge target's if there is one; else the best
 * by freshness, kind and nearness that no other bot is already after. */
static int IG_HuntTrace(bot_state_t *bs, vec3_t pos, float *time)
{
	int i, best = -1, nemesis;
	float t, score, bestscore = 0, memory = IG_HuntMemory();
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
	*time = 0;
	for (i = 0; i < maxclients && i < MAX_CLIENTS; i++) {
		if (i == bs->client) continue;
		if (!IG_LastEvidence(bs->client, i, p, &t)) continue;
		if (FloatTime() - t > memory) continue;
		VectorSubtract(p, bs->origin, dir);
		if (VectorLength(dir) < IG_PASS_DIST) continue;
		if (IG_TraceTaken(bs->client, i, p)) continue;
		score = (1 - (FloatTime() - t) / memory) / (1 + VectorLength(dir) / IG_NEAR_SCALE);
		if (ig_traces[bs->client][i].kind == IG_HEARD) score *= IG_HEARD_WEIGHT;
		if (score <= bestscore) continue;
		best = i;
		bestscore = score;
		*time = t;
		VectorCopy(p, pos);
	}
	return best;
}

/* High ground (IG-7): the map's high points (botlib/be_ig_hook.c), the
 * nearest by route the bot has not passed in IG_HG_FORGET seconds and no
 * other bot is heading for, among the three nearest at random for variety.
 * Passed through like a hunt goal. Replaces the random goal on the stack. */
int IG_HighGround(int max, int *areas, vec3_t *points);   /* botlib */

static void IG_HighGroundGoal(bot_state_t *bs)
{
	int areas[IG_HG_N], n, i, b, c = bs->client, best[3] = {-1, -1, -1}, pick;
	int besttime[3] = {0, 0, 0}, t;
	vec3_t points[IG_HG_N], d;
	bot_goal_t goal;

	if (!IG_RoamOn()) return;
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
	if (best[0] < 0) {
		IG_Debug(bs, IG_DBG_CHATTY, "no high ground to take, wandering");
		return;
	}
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
	ig_hunting[c] = 0;
	VectorCopy(points[pick], ig_hgpos[c]);
	BotAI_Print(PRT_MESSAGE, "ig: high ground %d point %d\n", c, pick);
	IG_Debug(bs, IG_DBG_CHATTY, "taking high ground %d", pick);
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
	ig_hgtarget[c] = 0;
	if (!IG_RoamOn() && IG_RevengeTarget(bs) < 0) return;
	if (!trap_BotGetTopGoal(bs->gs, &goal)) return;
	/* only AAS_RandomGoalArea's goal: GFL_ROAM and no item (an
	 * item_botroam goal is GFL_ROAM with an item number) */
	if (!(goal.flags & GFL_ROAM) || goal.number != 0) return;
	/* how often it hunts is the character's: 25% for the least aggressive,
	 * 85% for the most; the rest of the time the goal stays as it was (for
	 * now the random one, later high ground or a patrol). A grudge always. */
	if (IG_RevengeTarget(bs) < 0 &&
		random() > 0.25f + 0.6f * trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_AGGRESSION, 0, 1)) {
		IG_HighGroundGoal(bs);
		return;
	}
	enemy = IG_HuntTrace(bs, pos, &time);
	if (enemy < 0) {
		IG_HighGroundGoal(bs);
		return;
	}
	area = BotPointAreaNum(pos);
	if (!area) return;
	{
		static const char *how[] = { "", "saw", "heard", "was killed by" };
		char name[64];
		int k = ig_traces[c][enemy].kind;

		static int announced[MAX_CLIENTS];      /* 1 + the last target announced */

		if (announced[c] != enemy + 1 || IG_RevengeTarget(bs) == enemy)
		IG_Debug(bs, IG_RevengeTarget(bs) == enemy ? IG_DBG_RARE : IG_DBG_HUNT, "%shunting %s: %s them %.0f s ago",
				 IG_RevengeTarget(bs) == enemy ? "REVENGE: " : "",
				 ClientName(enemy, name, sizeof(name)), how[k >= 1 && k <= 3 ? k : 0],
				 FloatTime() - time);
		announced[c] = enemy + 1;
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

/* Diagnostic (cvar ig_movedebug, default 0): a line per think while the bot
 * seeks its long-term goal -- where it is, the goal, the travel type it is
 * executing and any failure -- to see why a bot mills around a spot. 2: only
 * failures and blocks. */
void IG_MoveDebug(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int level = trap_Cvar_VariableIntegerValue("ig_movedebug");

	if (!level) return;
	if (level >= 2 && !mr->failure && !mr->blocked) return;
	BotAI_Print(PRT_MESSAGE, "ig: move %d at %.0f %.0f %.0f area %d goal %.0f %.0f %.0f area %d travel %d fail %d blocked %d flags %d ent %d reach %d\n",
				bs->client, bs->origin[0], bs->origin[1], bs->origin[2], bs->areanum,
				goal->origin[0], goal->origin[1], goal->origin[2], goal->areanum,
				mr->traveltype, mr->failure, mr->blocked, mr->flags, mr->blockentity, trap_AAS_AreaReachability(bs->areanum));
}
