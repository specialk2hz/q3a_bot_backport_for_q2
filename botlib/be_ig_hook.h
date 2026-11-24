/* be_ig_hook.h -- the instagib fork's grapple wiring (be_ig_hook.c). */
#ifndef BE_IG_HOOK_H
#define BE_IG_HOOK_H

/* The entity weapon number a detected Q2 hook is given. Above every Q3 and
 * Q2 weapon number, so nothing else matches it. */
#define IG_GRAPPLE_WEAPON   1000

int IG_HookDetect(void);
int IG_IsHookEntity(int solid, int renderfx, int modelindex, char **names, int count);
/* IG-40: the bot's own hook, told by its hand (the hook's old_origin, which
 * the game re-projects to its owner every frame): 0 none, 1 flying, 2 still. */
int IG_OwnHook(const vec3_t eye, float speed, vec3_t hookpos);
int IG_GrappleState(const vec3_t origin, const vec3_t viewoffset, const vec3_t anchor);
void IG_AdjustGrappleCosts(void);
int IG_HighGround(int max, int *areas, vec3_t *points);
int IG_HookNear(const vec3_t point, float radius, vec3_t hookpos);
void IG_GrappleTravelNote(int client);
void IG_NoteHookFire(int client);
int IG_CanFireHook(int client);
float IG_GrappleStartDist(float dist);
/* IG-32 (libvar ig_climbrun): climbs fired on the run, let go at the top */
int IG_ClimbRun(void);
int IG_ClimbFireNow(const vec3_t origin, const vec3_t viewoffset, const vec3_t viewangles, const vec3_t start, const vec3_t aimdir, int passent);
int IG_ClimbLetGo(const vec3_t origin, const struct aas_reachability_s *reach);
void IG_NotePadFlight(int client, const vec3_t end);
int IG_PadFlight(int client, vec3_t end);
void IG_GrappleBlocked(int reachnum);
void IG_GrappleAim(const vec3_t eye, const vec3_t anchor, float lift, int passent, vec3_t viewdir);
void IG_GrappleWatchdog(int client);
/* IG-36 (libvar ig_climbaim): climbs hook the frame or the landing floor, then steer to the landing's middle */
struct aas_reachability_s;
int IG_ClimbAimOn(void);
int IG_ClimbAimReach(int client, int reachnum, const struct aas_reachability_s *reach, struct aas_reachability_s *out);
int IG_ClimbAimLetGo(int client, const vec3_t origin, const vec3_t velocity);
int IG_ClimbSteer(int client, const vec3_t origin, int grounded, int hooked, vec3_t dir);
/* IG-37 (libvar ig_drill): the climb drill */
void IG_ClimbAimForce(int client, int on);
void IG_ClimbAimClear(int client);
void IG_DrillSetActive(int client, int on);
int IG_ClimbAimSpot(int reachnum, vec3_t spot, float *oz);
int IG_ClimbAimPick(int client, int reachnum);
int IG_ClimbLookDir(int client, const vec3_t origin, vec3_t dir);
int IG_ClimbAimLetGoArc(int client, const vec3_t origin, const vec3_t velocity);
int IG_ClimbAimOverhead(int client);
int IG_ClimbLand(int client, const vec3_t origin, const vec3_t velocity, const vec3_t viewangles, const vec3_t viewoffset,
				 int grounded, int hooked, vec3_t movedir, vec3_t view, int *fire, int *release);
struct aas_reachability_s;
float IG_GrappleDist(int client, int reachnum, const vec3_t origin, const vec3_t viewoffset, const struct aas_reachability_s *reach, float last);

#endif
