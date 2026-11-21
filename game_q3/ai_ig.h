/* ai_ig.h -- the instagib fork's decision code (ai_ig_*.c). IG-7 and IG-8 in
 * the Colosseum fork's IG-SPECS.md. Every switch is a libvar the game sets,
 * off by default: off is upstream behaviour. */
#ifndef AI_IG_H
#define AI_IG_H

/* ai_ig_roam.c */
void IG_RoamThink(bot_state_t *bs);
void IG_RoamReplace(bot_state_t *bs, int tfl);
#define IG_SEEN     1
#define IG_HEARD    2
#define IG_KILLEDBY 3
void IG_Evidence(int bot, int enemy, const vec3_t pos);
void IG_EvidenceKind(int bot, int enemy, const vec3_t pos, int kind);
void IG_Debug(bot_state_t *bs, const char *fmt, ...);
int  IG_LastEvidence(int bot, int enemy, vec3_t pos, float *time);

/* ai_ig_revenge.c */
int  IG_RevengeTarget(bot_state_t *bs);
void IG_RevengeDeath(bot_state_t *bs, int attacker);
void IG_RevengeKill(bot_state_t *bs, int target);
int  IG_RevengeFindEnemy(bot_state_t *bs, int curenemy);

#endif
