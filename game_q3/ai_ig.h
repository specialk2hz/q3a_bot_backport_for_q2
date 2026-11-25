/* ai_ig.h -- the instagib fork's decision code (ai_ig_*.c). IG-7 and IG-8 in
 * the Colosseum fork's IG-SPECS.md. Every switch is a libvar the game sets,
 * off by default: off is upstream behaviour. */
#ifndef AI_IG_H
#define AI_IG_H

/* ai_ig_roam.c */
void IG_RoamThink(bot_state_t *bs);
int  IG_Peace(void);
void IG_RoamReplace(bot_state_t *bs, int tfl);
void IG_MoveDebug(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr);
#define IG_SEEN     1
#define IG_HEARD    2
#define IG_KILLEDBY 3
void IG_Evidence(int bot, int enemy, const vec3_t pos);
void IG_EvidenceKind(int bot, int enemy, const vec3_t pos, int kind);
#define IG_DBG_RARE   0
#define IG_DBG_HUNT   1
#define IG_DBG_CHATTY 2
void IG_Debug(bot_state_t *bs, int kind, const char *fmt, ...);
int  IG_LastEvidence(int bot, int enemy, vec3_t pos, float *time);

/* ai_ig_sling.c */
void IG_Slingshot(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *moveresult);
void IG_CombatHook(bot_state_t *bs);

/* ai_ig_pmove.c -- Quake II's player movement, for prediction (IG-40) */
typedef struct {
	vec3_t	org, vel, groundnormal;
	int		ground, jumpheld, passent, mask, ducked;
	float	gravity, friction, maxspeed, time;
	int		contents, everhit;		/* at the feet now; ever, over the run */
	int		blocked, hitplayer, jumped;	/* hit a wall / a player, jumped, over the run */
	/* the hook (IG-40 step 2): 0 none, 1 flying, 2 pulling, 3 hanging (OSP, inside 64) */
	int		hook, hookstyle, hooksky, hooklost;	/* style 0 OSP, 1 LMCTF; lost: missed or hit sky */
	vec3_t	hookpos, hookdir;
	float	hookfly, hookpull, hookflytime;
	int		steps;	/* pmoves run; the hook flies after every second (the game frame's end) */
	int		area, pad, teleport;	/* the navigation area now; entered a jump pad / teleporter (sticky) */
	int		jumps;					/* jumps made over the run */
} igpm_t;
void IG_PmoveHookFire(igpm_t *p, float pitch, float yaw);
void IG_PmoveHookRelease(igpm_t *p);
void IG_PmoveHookAt(igpm_t *p, int state, const vec3_t pos, const vec3_t dir);
void IG_PmoveInit(igpm_t *p, const vec3_t org, const vec3_t vel, int ground, int passent, int withplayers);
void IG_Pmove(igpm_t *p, float fmove, float smove, float upmove, float pitch, float yaw, int msec);
int  IG_PmoveTraces(void);

/* ai_ig_plan.c -- the movement planner (IG-40) */
int  IG_PlanOn(void);
int  IG_Plan(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr);
float IG_PlanAffinity(bot_state_t *bs);		/* ai_ig_sling.c */
void IG_PlanNode(int c, const char *node);
int  IG_PlanViewSet(int c);
void IG_PlanTick(bot_state_t *bs);
void IG_PlanGuard(bot_state_t *bs, bot_input_t *bi);
int  IG_PlanFireUnsafe(bot_state_t *bs, const vec3_t anchor, int hold);
int  IG_PlanHookUnsafe(bot_state_t *bs, int hold);
int  IG_PlanFight(bot_state_t *bs);
void IG_PlanTakeFlight(int c);
int  IG_PlanRepositionUrge(int c);		/* ai_ig_sling.c */

/* ai_ig_revenge.c */
int  IG_RevengeTarget(bot_state_t *bs);
void IG_RevengeDeath(bot_state_t *bs, int attacker);
void IG_RevengeKill(bot_state_t *bs, int target);
int  IG_RevengeFindEnemy(bot_state_t *bs, int curenemy);

#endif
