/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_revenge.c -- a grudge (IG-8).
 *
 * A bot killed twice by the same player within ig_revenge_window seconds
 * (default 60) takes that player as its nemesis, until it kills them or
 * ig_revenge_time seconds pass (default 20). Meanwhile:
 *   - its hunt goal is the nemesis's last trace (ai_ig_roam.c), even when
 *     ig_roam is off;
 *   - with the nemesis in view, it is the enemy, over a nearer one
 *     (BotFindEnemy).
 * It knows only what it perceived: where it was killed from, and what it
 * sees and hears after. Lines "ig: revenge <bot> on <nemesis>", "ig: revenge
 * kill ..." and "ig: revenge expired ..." are what igbench counts.
 */

#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"

#define IG_REVENGE_FOV 160

static int   ig_lastkiller[MAX_CLIENTS];      /* 1 + client, 0: none */
static float ig_lastkilled[MAX_CLIENTS];      /* when */
static int   ig_nemesis[MAX_CLIENTS];         /* 1 + client, 0: none */
static float ig_nemesis_until[MAX_CLIENTS];

static int IG_RevengeOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_revenge") != 0;
}

static float IG_Libvar(const char *name, float def)
{
	int v = trap_Cvar_VariableIntegerValue(name);
	return v > 0 ? v : def;
}

int IG_RevengeTarget(bot_state_t *bs)
{
	int c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS || !ig_nemesis[c]) return -1;
	/* expired, or a new map (the clock went back) */
	if (FloatTime() > ig_nemesis_until[c] ||
		ig_nemesis_until[c] - FloatTime() > IG_Libvar("ig_revenge_time", 20) + 1) {
		BotAI_Print(PRT_MESSAGE, "ig: revenge expired %d on %d\n", c, ig_nemesis[c] - 1);
		ig_nemesis[c] = 0;
		return -1;
	}
	return ig_nemesis[c] - 1;
}

/* From the obituary in BotCheckEvents (ai_dmq3.c): the bot was killed. */
void IG_RevengeDeath(bot_state_t *bs, int attacker)
{
	int c = bs->client;
	aas_entityinfo_t entinfo;

	if (!IG_RevengeOn() || c < 0 || c >= MAX_CLIENTS) return;
	if (attacker < 0 || attacker >= MAX_CLIENTS || attacker == c) return;
	/* who killed it, and from where, it knows */
	BotEntityInfo(attacker, &entinfo);
	if (entinfo.valid) IG_Evidence(c, attacker, entinfo.origin);
	if (ig_lastkiller[c] == attacker + 1 &&
		FloatTime() - ig_lastkilled[c] <= IG_Libvar("ig_revenge_window", 60) &&
		ig_nemesis[c] != attacker + 1) {
		ig_nemesis[c] = attacker + 1;
		ig_nemesis_until[c] = FloatTime() + IG_Libvar("ig_revenge_time", 20);
		BotAI_Print(PRT_MESSAGE, "ig: revenge %d on %d\n", c, attacker);
		{ char name[64]; IG_Debug(bs, IG_DBG_RARE, "grudge: %s killed me twice", ClientName(attacker, name, sizeof(name))); }
	}
	ig_lastkiller[c] = attacker + 1;
	ig_lastkilled[c] = FloatTime();
}

/* From the obituary: the bot killed someone. */
void IG_RevengeKill(bot_state_t *bs, int target)
{
	int c = bs->client;

	if (c < 0 || c >= MAX_CLIENTS || !ig_nemesis[c]) return;
	if (ig_nemesis[c] != target + 1) return;
	BotAI_Print(PRT_MESSAGE, "ig: revenge kill %d on %d\n", c, target);
	{ char name[64]; IG_Debug(bs, IG_DBG_RARE, "revenge on %s", ClientName(target, name, sizeof(name))); }
	ig_nemesis[c] = 0;
	/* a settled score is not a grudge */
	ig_lastkiller[c] = 0;
}

/* From the top of BotFindEnemy: -1 carry on as usual; 0 keep the current
 * enemy (it is the nemesis); 1 the nemesis is the enemy now. */
int IG_RevengeFindEnemy(bot_state_t *bs, int curenemy)
{
	int n = IG_RevengeTarget(bs);
	aas_entityinfo_t entinfo;

	if (n < 0) return -1;
	if (curenemy == n) return 0;
	BotEntityInfo(n, &entinfo);
	if (!entinfo.valid || EntityIsDead(&entinfo)) return -1;
	if (BotEntityVisible(bs->entitynum, bs->eye, bs->viewangles, IG_REVENGE_FOV, n) <= 0) return -1;
	bs->enemy = n;
	bs->enemysight_time = FloatTime();
	bs->enemysuicide = 0;
	bs->enemydeath_time = 0;
	bs->enemyvisible_time = FloatTime();
	return 1;
}
