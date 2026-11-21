/* be_ig_hook.h -- the instagib fork's grapple wiring (be_ig_hook.c). */
#ifndef BE_IG_HOOK_H
#define BE_IG_HOOK_H

/* The entity weapon number a detected Q2 hook is given. Above every Q3 and
 * Q2 weapon number, so nothing else matches it. */
#define IG_GRAPPLE_WEAPON   1000

int IG_HookDetect(void);
int IG_IsHookEntity(int solid, int renderfx, int modelindex, char **names, int count);
int IG_GrappleState(const vec3_t origin, const vec3_t viewoffset, const vec3_t anchor);
void IG_AdjustGrappleCosts(void);
int IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void IG_GrappleTravelNote(int client);
float IG_GrappleStartDist(float dist);
void IG_GrappleBlocked(int reachnum);
void IG_GrappleAim(const vec3_t eye, const vec3_t anchor, float lift, int passent, vec3_t viewdir);
void IG_GrappleWatchdog(int client);
struct aas_reachability_s;
float IG_GrappleDist(int client, int reachnum, const vec3_t origin, const vec3_t viewoffset, const struct aas_reachability_s *reach, float last);

#endif
