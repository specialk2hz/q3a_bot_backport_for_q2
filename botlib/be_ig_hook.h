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
void IG_GrappleTravelNote(int client);
void IG_GrappleAim(const vec3_t eye, const vec3_t anchor, float lift, int passent, vec3_t viewdir);
void IG_GrappleWatchdog(int client);
float IG_GrappleDist(const vec3_t origin, const vec3_t viewoffset, const vec3_t anchor);

#endif
