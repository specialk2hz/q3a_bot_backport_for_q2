/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* ai_ig_plan.c -- the movement planner (IG-40, libvar ig_plan).
 *
 * One decision a think instead of a controller per trick. From the bot's
 * state now, a handful of OPTIONS -- run the route, strafe-hop, slingshot to
 * one of a few anchors along the route, hold or let go of the hook out -- are
 * each flown for PLAN_THINKS thinks in Quake II's own movement
 * (ai_ig_pmove.c, which matches the game to a unit or two), and SCORED by
 * where they leave the bot:
 *
 *   cost = time flown + the navigation's travel time from there to the goal
 *          - a bonus for speed carried along the route (it saves time later)
 *          + a penalty for hitting a wall fast
 *   lava or slime touched, or no route from where it ends (a pit): out.
 *
 * The bot does the first think of an option picked among those within
 * PLAN_MARGIN of the best -- by chance, a hook option weighted by the bot's
 * hook affinity -- and plans again next think. So a trick is chosen where it
 * pays, varies in how it is done, and never flies into lava it could see.
 *
 * Execution goes through the same input function the simulation uses
 * (IGX_Input), so what was scored is what is sent. Hooks are aimed by a
 * flick (the view snaps to the anchor) and fired on the next think: the game
 * fires along the view of the last usercmd.
 *
 * The planner steps back (returns 0, the stock route movement stands) when
 * the route is doing something it does not plan -- a ladder, a lift, a climb
 * route, a jump pad, swimming -- and nothing of its own is under way.
 */
#include "../botlib/ai_q2_compat.h"
#include "ai_main.h"
#include "ai_dmq3.h"
#include "ai_ig.h"
#include "chars.h"
#include "../botlib/be_ig_hook.h"

int AAS_AreaRouteToGoalArea(int areanum, vec3_t origin, int goalareanum, int travelflags, int *traveltime, int *reachnum);
void AAS_ReachabilityFromNum(int num, struct aas_reachability_s *reach);
int AAS_AreaTravelTimeToGoalArea(int areanum, vec3_t origin, int goalareanum, int travelflags);
int AAS_PointAreaNum(vec3_t point);
int AAS_PointContents(vec3_t point);
float LibVarValue(const char *var_name, const char *value);
void EA_IGMove(int client, vec3_t dir, float speed, int jump, int crouch);
bot_goal_t *IG_PadDrillGoal(int client);	/* IG-43, be_ai_move.c */
void IG_PadDrillNoteTop(int client);

#define PLAN_THINKS		12		/* 1.2 s ahead */
#define PLAN_MARGIN		0.06f	/* options this close to the best are all fair picks (s) */
#define PLAN_KEEP		0.04f	/* the option under way gets this much benefit of the doubt (s) */
#define PLAN_COMMIT		0.25f	/* a flick is a promise: its fire, next think, unless that turned bad (s) */
#define PLAN_SPEEDBONUS	0.35f	/* s saved per 300 u/s carried along the route beyond running speed */
/* A hook has a price: a seasoned player does not hook for a 2% gain. In
 * seconds, scaled by the bot's hook affinity (a grapple lover pays less):
 * libvar ig_planhookcost (s at the middle affinity, 0.55). */
#define PLAN_COAST		15		/* up to 1.5 s more, running the route, until the extra speed is spent */
#define PLAN_MAXANCH	36	/* IG-47: + 18 lip anchors (floor, ceiling) before a momentum jump */
#define PLAN_MAXOPT		48	/* IG-47: room for the lip anchors' slingshots */

enum { PO_RUN, PO_HOP, PO_SLING, PO_FIRE, PO_HOLD, PO_LETGO, PO_LETGOHOP, PO_HOLDHOP, PO_LANDHOP };
static const char *po_names[] = { "run", "hop", "sling", "fire", "hold", "letgo", "letgohop", "holdhop", "landhop" };

typedef struct {
	int		kind;
	int		hop;		/* strafe-hop chain for the whole run */
	int		jump0;		/* jump on the first think */
	int		anchor;		/* flick onto anchors[anchor] on the first think; -1 none */
	int		fire;		/* think to fire at; -1 none */
	int		relafter;	/* let go this many thinks after the hook bites; 0 keep it */
	int		letgo;		/* let go on the first think */
	int		wallhit;	/* its simulation runs into a wall (200+ u/s lost) */
	int		landhop;	/* jump pressed coming down, so the landing jumps at once -- once, then run */
	float	hookmul;	/* the hook price times this: by the anchor's class */
	float	cost;
	int		ok;
} igopt_t;

typedef struct {
	bot_state_t	*bs;
	int			goalarea, tfl, jumpland, canfire;
	vec3_t		goalorg;
	vec3_t		anchors[PLAN_MAXANCH];
	vec3_t		anchorview[PLAN_MAXANCH];	/* pitch, yaw from the eye now */
	int			anchorclass[PLAN_MAXANCH];	/* IGA_SLING, IGA_UP, IGA_SIDE */
	float		anchornz[PLAN_MAXANCH];		/* the surface's normal z: > 0.7 floor */
	bot_goal_t	*goal;
	int			numanchors;
	vec3_t		lastview;					/* the view the game holds now (last usercmd) */
	int			hook;						/* own hook now: 0, 1 flying, 2 still */
	float		hookcost;					/* the price of firing a hook, s */
	float		flow;						/* s per 300 u/s of average speed kept (ig_planflow) */
	float		pullfloor;					/* s per 50 ms pulled along the floor (ig_planpullfloor) */
	float		strafeskill;				/* IG_StrafeSkill: the share of a strafe's most speed */
	int			released, reljumps;			/* in a simulation: the hook let go, and the jumps made by then */
	int			slinghop;					/* slingshots are flown with Billy's finish (ig_planslinghop) */
	vec3_t		hookpos, hookdir;
	int			haslook;					/* IGX_LookAhead found a point to curve toward */
	int			hasjump;					/* IG-47: a momentum jump ahead on the route (IGX_LookAhead) */
	vec3_t		jumpfrom, jumpdir;			/* its take-off and its direction (level, unit) */
	int			nearonly;					/* IGX_AnchorTry: only anchors short of the jump's edge */
	int			climbrn;					/* IG-40 climbs: a grapple route ahead the planner climbs itself */
	int			nclimb;
	vec3_t		climbanch[3];				/* its anchors (bspc's, the climb table's frame and floor) */
	float		turnahead;					/* the route's turn within a hop's reach, degrees */
	vec3_t		look, lookfrom;
} igx_t;

static struct {
	int		active;			/* something of ours under way: a hop, a sling, our hook */
	int		kind, anchorset;
	vec3_t	anchor, flickview;
	float	flicktime;
	int		hookout;		/* we fired the hook that is out */
	int		anchorclass;	/* the flicked anchor's class */
	int		landhop;		/* a landing hop decided: jump held coming down until it lands */
	int		padfired;		/* IG-40 pad top: this pad flight has had its slingshot */
	int		fightside;		/* the strafe side in a fight's air time, swapped every think */
	float	anchornz;
	vec3_t	firedir;		/* ...along this: the flight a flying hook keeps */
	int		chain;
	float	lastlog;
	float	steered;	/* the think the planner set the input itself */
	float	viewset;	/* the think the planner set the view (a flick, or holding one) */
	/* this think's decision: the AI may run two nodes in one think (a node
	 * switch), and each calls the planner -- the second gets the first's */
	float	decided;
	int		ret, setmove, jump;
	vec3_t	wish, view;
	/* IG-47: the last momentum jump seen ahead, kept through its flight */
	float	jumpuntil;
	vec3_t	jumpfrom, jumpdir;
} igp_state[MAX_CLIENTS];

/* ------------------------------------------------------------ diagnostics */

static char  igp_node[MAX_CLIENTS][24];
static float igp_lastplan[MAX_CLIENTS];

/* From BotRecordNodeSwitch: the AI node the bot is in now. */
void IG_PlanNode(int c, const char *node)
{
	if (c >= 0 && c < MAX_CLIENTS) Q_strncpyz(igp_node[c], node, sizeof(igp_node[c]));
}

/* True when the planner set the view this think: the adapter's roaming-view
 * clamp (be_interface_q2.c) must not turn a flick off its anchor -- it did,
 * and 64% of the planner's hooks flew past what they were aimed at. */
int IG_PlanViewSet(int c)
{
	return c >= 0 && c < MAX_CLIENTS && IG_PlanOn() && FloatTime() - igp_state[c].viewset < 0.001f;
}

/* Every think (be_interface_q2.c), ig_plandebug 2: what the planner saw. */
void IG_PlanTick(bot_state_t *bs)
{
	int c = bs->client, hs;
	vec3_t eye, hp;

	if (c < 0 || c >= MAX_CLIENTS || trap_Cvar_VariableIntegerValue("ig_plandebug") != 2) return;
	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	hs = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), hp);
	BotAI_Print(PRT_MESSAGE, "igpl %d hook %d ours %d planned %.1f node %s at %.0f %.0f %.0f v %.0f\n", c, hs,
				igp_state[c].hookout, FloatTime() - igp_lastplan[c], igp_node[c],
				bs->cur_ps.origin[0], bs->cur_ps.origin[1], bs->cur_ps.origin[2], VectorLength(bs->cur_ps.velocity));
}

/* ---------------------------------------------------------------- route */

int IG_FastJumpTfl(int client, const vec3_t vel);	/* be_ig_jump.c, IG-47 */

/* The route's way from org. *special: the route's next step there is one
 * the simulation does not model -- a gap jump, a barrier jump, a ladder, a
 * lift, a climb, a pad, a teleporter: the stock movement takes those, and a
 * simulated run would walk off the gap's edge (brutalo, q2rdm4: upper level
 * time fell 4-7 points while it did). */
static float IGX_HSpeed(bot_state_t *bs);
static int IGX_RouteDirS(igx_t *x, const vec3_t org, int area, vec3_t dir, int *special)
{
	int tt, rn, tv;
	aas_reachability_t r;
	vec3_t t, o;

	if (special) *special = 0;
	if (!area) return 0;
	VectorCopy(org, o);
	/* the look-ahead point (IGX_LookAhead), until it is reached or passed */
	if (x->haslook) {
		vec3_t a, b;

		VectorSubtract(x->look, o, a);
		VectorSubtract(x->look, x->lookfrom, b);
		a[2] = b[2] = 0;
		if (DotProduct(a, b) > 0 && VectorLength(a) > 64) {
			VectorCopy(a, dir);
			return VectorNormalize(dir) > 1;
		}
	}
	if (area == x->goalarea) {
		VectorCopy(x->goalorg, t);
	} else {
		if (!AAS_AreaRouteToGoalArea(area, o, x->goalarea, x->tfl, &tt, &rn) || !rn) return 0;
		AAS_ReachabilityFromNum(rn, &r);
		tv = r.traveltype & TRAVELTYPE_MASK;
		if (special && tv != TRAVEL_WALK && tv != TRAVEL_WALKOFFLEDGE && tv != TRAVEL_STRAFEJUMP &&
			!(tv == TRAVEL_GRAPPLEHOOK && x->climbrn)) *special = 1;	/* IG-47: momentum jumps are flown by the sim; IG-40: so are the planner's climbs */
		VectorCopy(r.start, t);
		/* at the reachability's start already: head across it. A momentum jump
		 * (IG-47) heads for its landing at once: the sim flies it from wherever
		 * the run reaches the edge, and steering at the take-off first ran
		 * q2rdm11's walkway-end bots into the pillar beside the gap */
		if (tv == TRAVEL_STRAFEJUMP ||
			(r.start[0] - o[0]) * (r.start[0] - o[0]) + (r.start[1] - o[1]) * (r.start[1] - o[1]) < 40 * 40)
			VectorCopy(r.end, t);
	}
	VectorSubtract(t, o, dir);
	dir[2] = 0;
	return VectorNormalize(dir) > 1;
}

static int IGX_RouteDir(igx_t *x, const vec3_t org, int area, vec3_t dir)
{
	return IGX_RouteDirS(x, org, area, dir, NULL);
}

/* Curving through turns (Billy, watching roaming bots: "hitting the end of the
 * walls before changing directions instead of curving"). The route's way was
 * the next step's start -- a corner -- until within 40 units of it. Once a
 * think: walk the route on up to ig_planlookdist (500) units through plain
 * walking steps, and take the farthest point reached in a straight line: the
 * body clears (a trace raised a step's height) and there is floor under the
 * line, no lava or slime. The decision and the simulation steer at it until it
 * is reached or passed (IGX_RouteDirS). ig_planlook 0: the old way. */
static int IGX_LineClear(bot_state_t *bs, const vec3_t a, const vec3_t b)
{
	vec3_t mins = {-16, -16, -6}, maxs = {16, 16, 32}, p, q, d;
	bsp_trace_t tr;
	float len, f;

	if (fabs(b[2] - a[2]) > 64) return 0;
	BotAI_Trace(&tr, (float *) a, mins, maxs, (float *) b, bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP);
	if (tr.startsolid || tr.fraction < 1) return 0;
	VectorSubtract(b, a, d);
	len = VectorLength(d);
	for (f = 48; f < len; f += 48) {
		VectorMA(a, f / len, d, p);
		VectorCopy(p, q);
		q[2] -= 24 + 64;
		BotAI_Trace(&tr, p, NULL, NULL, q, bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP | CONTENTS_LAVA | CONTENTS_SLIME);
		if (tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME))) return 0;
	}
	return 1;
}

/* IG-47: a momentum jump seen ahead stays in force until the bot is down on
 * the far side (1.5 s at most), so no hook past its edge mid-flight either */
static void IGX_JumpKeep(igx_t *x)
{
	int c = x->bs->client;
	if (c < 0 || c >= MAX_CLIENTS) return;
	if (x->hasjump) {
		igp_state[c].jumpuntil = FloatTime() + 1.5f;
		VectorCopy(x->jumpfrom, igp_state[c].jumpfrom);
		VectorCopy(x->jumpdir, igp_state[c].jumpdir);
	} else if (FloatTime() < igp_state[c].jumpuntil) {
		if (x->bs->cur_ps.groundEntityNum != ENTITYNUM_NONE) {
			vec3_t pa; VectorSubtract(x->bs->cur_ps.origin, igp_state[c].jumpfrom, pa); pa[2] = 0;
			if (DotProduct(pa, igp_state[c].jumpdir) > 32) { igp_state[c].jumpuntil = 0; return; }	/* down past the edge: done */
		}
		x->hasjump = 1;
		VectorCopy(igp_state[c].jumpfrom, x->jumpfrom);
		VectorCopy(igp_state[c].jumpdir, x->jumpdir);
	}
}

/* IG-40 climbs (libvar ig_planclimb; BOT-TRICKS T9). Billy: bots "completely
 * stop moving and grapple and pull to move when a seasoned player would be
 * moving in the direction that they want to grapple", and with a slow hook he
 * prefires. The stock grapple travel walks to the route's start, stops, aims,
 * fires (71% of climbs from a standstill); loosened it lost climbs (IG-32).
 * Here a grapple route within ig_planclimbdist (600) along the route is the
 * planner's: its anchors become slingshot targets, simulated from where the
 * bot is, moving, with the hook's own flight (a slow hook fires earlier) and
 * pull; replanning while hooked lets go when the sim says. The stock travel
 * stands down while the planner notes it (IG_PlanClimbOwns); after
 * ig_planclimbgive (3) s on one climb without firing, it is the stock's. */
static int igpc_rn[MAX_CLIENTS], igpc_fired[MAX_CLIENTS];
static float igpc_since[MAX_CLIENTS];
static void IGX_FindClimb(igx_t *x, const vec3_t org)
{
	int c = x->bs->client, k, ar, tt, rn;
	float dist = 0, maxd;
	vec3_t o;
	aas_reachability_t r;

	x->climbrn = 0; x->nclimb = 0;
	if (!x->canfire || !LibVarValue("ig_planclimb", "0") || c < 0 || c >= MAX_CLIENTS || !x->goalarea) return;
	/* not yet with LMCTF's hook: 28 maps, hooks that climbed -2.7 pts (its pull is misjudged) */
	if (LibVarValue("ig_hookstyle", "0") && !LibVarValue("ig_planclimblmctf", "0")) return;
	maxd = LibVarValue("ig_planclimbdist", "600");
	VectorCopy(org, o);
	ar = AAS_PointAreaNum(o);
	for (k = 0; k < 6 && ar && ar != x->goalarea && dist < maxd; k++) {
		if (!AAS_AreaRouteToGoalArea(ar, o, x->goalarea, x->tfl, &tt, &rn) || !rn) return;
		AAS_ReachabilityFromNum(rn, &r);
		if ((r.traveltype & TRAVELTYPE_MASK) == TRAVEL_GRAPPLEHOOK) {
			if (igpc_rn[c] != rn && FloatTime() - igpc_since[c] > LibVarValue("ig_planclimbgive", "3")) { igpc_rn[c] = rn; igpc_since[c] = FloatTime(); igpc_fired[c] = 0; }
			else if (igpc_rn[c] != rn) return;	/* another climb inside the stretch: the stock's (a route flipping between two climbs restarted the clock: Billy saw a bot sit) */
			if (!igpc_fired[c] && FloatTime() - igpc_since[c] > LibVarValue("ig_planclimbgive", "3")) return;	/* the stock's */
			x->climbrn = rn;
			x->nclimb = IG_ClimbAnchorList(rn, &r, x->climbanch);
			return;	/* owned (IG_PlanClimbNote) only on a think the planner moves the bot */
		}
		if ((r.traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) return;
		dist += Distance(o, r.end);
		VectorCopy(r.end, o);
		ar = r.areanum;
	}
}

static void IGX_LookAhead(igx_t *x, const vec3_t org, int area)
{
	vec3_t pts[16], o, a, b, jumpfrom;
	int n = 0, k, tt, rn, ar = area, tv, jumpland = -1;
	aas_reachability_t r;
	float dist = 0, maxd = LibVarValue("ig_planlookdist", "500");

	x->haslook = 0;
	x->hasjump = 0;
	if (!LibVarValue("ig_planlook", "1") || !ar) return;
	VectorCopy(org, o);
	for (k = 0; k < 8 && n < 15 && dist < maxd; k++) {
		if (ar == x->goalarea) { VectorCopy(x->goalorg, pts[n]); n++; break; }
		if (!AAS_AreaRouteToGoalArea(ar, o, x->goalarea, x->tfl, &tt, &rn) || !rn) break;
		AAS_ReachabilityFromNum(rn, &r);
		tv = r.traveltype & TRAVELTYPE_MASK;
		if (tv == TRAVEL_STRAFEJUMP) {	/* IG-47: curve toward a momentum jump's landing */
			VectorCopy(r.start, jumpfrom); VectorCopy(r.end, pts[n]); n++; jumpland = n - 1;
			x->hasjump = 1; VectorCopy(r.start, x->jumpfrom);
			VectorSubtract(r.end, r.start, x->jumpdir); x->jumpdir[2] = 0; VectorNormalize(x->jumpdir);
			{	/* the take-off may be the start of the run-up: the edge is where the floor ends */
				int q; vec3_t p0, p1; bsp_trace_t gt;
				for (q = 0; q < 40; q++) {
					VectorMA(r.start, q * 16, x->jumpdir, p0);
					VectorCopy(p0, p1); p1[2] -= 64;
					BotAI_Trace(&gt, p0, NULL, NULL, p1, x->bs->entitynum, CONTENTS_SOLID);
					if (gt.fraction >= 1) break;
					VectorCopy(p0, x->jumpfrom);
				}
			}
			break;
		}
		if (tv != TRAVEL_WALK) break;
		VectorCopy(r.start, pts[n]); n++;
		VectorCopy(r.end, pts[n]); n++;
		VectorSubtract(r.end, o, a);
		dist += VectorLength(a);
		VectorCopy(r.end, o);
		ar = r.areanum;
	}
	/* how sharply the route turns within a hop's reach (Billy, his corners
	 * recorded 2026-11-25: turns of 50-70 degrees he hops through, curving in
	 * the air; 85-125 he stops hopping, turns his view on the last hop with the
	 * side key only, lands and turns on the ground) */
	x->turnahead = 0;
	{
		vec3_t h0, h1, p0;
		float reach = IGX_HSpeed(x->bs) * 0.6f + 64, along = 0;
		int m;

		VectorSet(h0, x->bs->cur_ps.velocity[0], x->bs->cur_ps.velocity[1], 0);
		if (VectorNormalize(h0) > 100) {
			VectorCopy(org, p0);
			for (m = 0; m < n; m++) {
				VectorSubtract(pts[m], p0, h1);
				h1[2] = 0;
				if (VectorLength(h1) < 24) continue;
				along += VectorNormalize(h1);
				{
					float t = RAD2DEG(acos(DotProduct(h0, h1) > 1 ? 1 : DotProduct(h0, h1) < -1 ? -1 : DotProduct(h0, h1)));
					if (t > x->turnahead) x->turnahead = t;
				}
				VectorCopy(pts[m], p0);
				if (along > reach) break;
			}
		}
	}
	VectorCopy(org, a);
	a[2] += 18;
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2 && jumpland >= 0) {
		vec3_t bb; VectorCopy(jumpfrom, bb); bb[2] += 18;
		BotAI_Print(PRT_MESSAGE, "iglook %d n %d jumpland %d at %.0f %.0f %.0f dist %.0f clear %d\n", x->bs->client, n, jumpland, pts[jumpland][0], pts[jumpland][1], pts[jumpland][2],
			sqrt((pts[jumpland][0] - org[0]) * (pts[jumpland][0] - org[0]) + (pts[jumpland][1] - org[1]) * (pts[jumpland][1] - org[1])), IGX_LineClear(x->bs, a, bb));
	}
	for (k = n - 1; k >= 1; k--) {		/* the farthest in a straight line (one step ahead needs no look-ahead) */
		VectorSubtract(pts[k], org, b);
		b[2] = 0;
		/* a momentum jump's landing from up to ig_planlookjump (1200): lined up
		 * from the start of the run, not 500 units out (q2rdm11's walkway end) */
		if (VectorLength(b) > (k == jumpland ? LibVarValue("ig_planlookjump", "1200") : maxd + 64) || VectorLength(b) < 96) continue;
		VectorCopy(pts[k], b);
		b[2] += 18;
		/* a jump landing: the flight from the take-off was proven at map load,
		 * so only the run to the take-off needs a clear line */
		if (k == jumpland) { VectorCopy(jumpfrom, b); b[2] += 18; }
		if (!IGX_LineClear(x->bs, a, b)) continue;
		VectorCopy(pts[k], x->look);
		VectorCopy(org, x->lookfrom);
		x->haslook = 1;
		return;
	}
}

/* In the air: the wish that keeps the most speed while turning toward the
 * route (Q2's air accel 1 adds speed only within acos(285/speed) of the
 * velocity -- the strafe-jump angle). */
/* The bot's strafe-jump skill (Billy: "some great bots and some not so great
 * in the game at once"): the share of the most speed a strafe can add. From
 * the character file's characteristic 50 when it is set, else from its
 * jumper and attack skill (0.45 .. 0.90). libvar ig_planstrafeskill 0..1
 * sets it for every bot (testing). */
#define CHARACTERISTIC_IG_STRAFE	50
int Characteristic_IGFloat(int character, int index, float *out);
float IG_StrafeSkill(bot_state_t *bs)
{
	float v = LibVarValue("ig_planstrafeskill", "2"), j, a;	/* above 1: not set (libvars take no negatives) */

	if (v <= 1) return v;
	if (Characteristic_IGFloat(bs->character, CHARACTERISTIC_IG_STRAFE, &v) && v > 0 && v <= 1) return v;
	j = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_JUMPER, 0, 1);
	a = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_ATTACK_SKILL, 0, 1);
	return 0.45f + 0.45f * (0.5f * j + 0.5f * a);
}

/* The strafe angle for a skill. A bot's wish holds for a whole think, two
 * 50 ms pmoves; the one-step best angle, acos(285/speed), adds 15 u/s in the
 * first and much less in the second, the velocity having turned toward the
 * wish (measured: +12 a think where one-step theory says +24). So the angle
 * is the best for the two steps together (IGX_Strafe2, a table by speed made
 * once), and a skill below 1 is the wider angle that adds that share of it --
 * still a strafe, still the swing, less speed. */
static float IGX_Strafe2(float v, float phi)
{
	float vx = v, vy = 0, wx = cos(phi), wy = sin(phi), add, acc;
	int i;

	for (i = 0; i < 2; i++) {
		add = 300 - (vx * wx + vy * wy);
		if (add <= 0) continue;
		acc = add < 15 ? add : 15;
		vx += acc * wx;
		vy += acc * wy;
	}
	return sqrt(vx * vx + vy * vy) - v;
}

#define IGX_STRAFE_V0	300
#define IGX_STRAFE_N	131		/* 300 .. 1600 u/s by 10 */
static float igx_bestphi[IGX_STRAFE_N], igx_bestgain[IGX_STRAFE_N];

static float IGX_StrafeAngle(float sp, float skill)
{
	static int made;
	float target, lo, hi, mid, best;
	int i, k;

	if (!made) {
		for (i = 0; i < IGX_STRAFE_N; i++) {
			float v = IGX_STRAFE_V0 + 10 * i, g, bp = 0, bg = -1;

			for (k = 0; k <= 900; k++) {		/* 0.1-degree steps */
				g = IGX_Strafe2(v, DEG2RAD(k * 0.1f));
				if (g > bg) { bg = g; bp = DEG2RAD(k * 0.1f); }
			}
			igx_bestphi[i] = bp;
			igx_bestgain[i] = bg;
		}
		made = 1;
	}
	if (sp < IGX_STRAFE_V0) sp = IGX_STRAFE_V0;
	i = (int)((sp - IGX_STRAFE_V0) / 10 + 0.5f);
	if (i >= IGX_STRAFE_N) i = IGX_STRAFE_N - 1;
	best = igx_bestphi[i];
	if (skill >= 0.999f) return best;
	/* the wider angle where the gain is skill times the best (the gain falls
	 * off steadily from the best angle toward 90 degrees) */
	target = skill * igx_bestgain[i];
	lo = best;
	hi = DEG2RAD(89.0f);
	for (k = 0; k < 14; k++) {
		mid = 0.5f * (lo + hi);
		if (IGX_Strafe2(sp, mid) > target) lo = mid;
		else hi = mid;
	}
	return 0.5f * (lo + hi);
}

static void IGX_AirWish(const vec3_t vel, const vec3_t route, vec3_t wish, float skill)
{
	vec3_t vh;
	float sp, theta, a, side, c, s;

	VectorSet(vh, vel[0], vel[1], 0);
	sp = VectorNormalize(vh);
	if (sp < 200) {
		VectorCopy(route, wish);
		return;
	}
	theta = IGX_StrafeAngle(sp, skill);
	a = acos(DotProduct(vh, route) > 1 ? 1 : DotProduct(vh, route) < -1 ? -1 : DotProduct(vh, route));
	/* Always at the strafe-jump angle, on the route's side -- also when
	 * the route runs straight ahead: wishing along the velocity above 300
	 * adds nothing in Q2, and that was most hops (planner chains gained 2
	 * u/s a second, Billy's route8 session 58). The side follows the route,
	 * so a straight run swings across it, the way he strafes. Under 300
	 * u/s, or with the route more than 90 degrees off, the route itself. */
	if (sp < 300 || a > M_PI / 2) {
		VectorCopy(route, wish);
		return;
	}
	side = vh[0] * route[1] - vh[1] * route[0] >= 0 ? 1 : -1;
	c = cos(theta);
	s = sin(theta) * side;
	wish[0] = vh[0] * c - vh[1] * s;
	wish[1] = vh[0] * s + vh[1] * c;
	wish[2] = 0;
}

/* --------------------------------------------------------------- inputs */

typedef struct {
	vec3_t	wish;
	int		jump, crouch;
	vec3_t	view;		/* pitch, yaw sent this think */
	int		fire, letgo;
} igin_t;

/* The input of option `o` at think t, for a body in state pm (sim or real).
 * routedir: the route's way from here. latch: think the hook bit (-1). */
/* A landing hop on a flight with no hook in it (a strafe jump's, a pad's): it
 * pressed jump only after a hook's let-go, so on a plain flight "hop the
 * landing" was "run" -- scored the same, and the bot landed flat: 808 -> 396
 * u/s on Billy's walkway (ig_planplainlandhop 0: the old way). */
static int IGX_PlainLandHop(const igx_t *x, const igopt_t *o)
{
	return o->kind == PO_LANDHOP && !x->hook && LibVarValue("ig_planplainlandhop", "1") != 0;
}

static void IGX_Input(igx_t *x, const igopt_t *o, int t, const igpm_t *pm, const vec3_t routedir,
					  int latch, igin_t *in)
{
	Com_Memset(in, 0, sizeof(*in));
	VectorCopy(x->lastview, in->view);
	if (t == 0 && o->anchor >= 0) VectorCopy(x->anchorview[o->anchor], in->view);
	if (t == 0 && o->letgo) in->letgo = 1;
	if (t == o->fire) in->fire = 1;
	if (o->relafter > 0 && latch >= 0 && t - latch >= o->relafter) in->letgo = 1;
	/* a slingshot lets go before the pull drags it onto the floor (Billy, 5
	 * of 5): coming down, and the floor within a think's fall */
	if ((o->kind == PO_SLING || o->kind == PO_FIRE) && latch >= 0 && t > latch && pm->hook >= 2 && !pm->ground && pm->vel[2] < 0) {
		bsp_trace_t tr;
		vec3_t down, mins = {-16, -16, -24}, maxs = {16, 16, 32}, org;

		VectorCopy(pm->org, org);
		VectorMA(org, 0.12f, pm->vel, down);
		down[2] -= 4;
		BotAI_Trace(&tr, org, mins, maxs, down, x->bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP);
		if (tr.fraction < 1 && tr.plane.normal[2] > 0.7f) in->letgo = 1;
	}

	if (pm->ground) {
		VectorCopy(routedir, in->wish);
		in->jump = (t == 0 && o->jump0) || o->hop || (o->landhop && (x->released || in->letgo || IGX_PlainLandHop(x, o)) && pm->jumps == x->reljumps && (in->letgo || pm->hook < 2));
	} else {
		if (pm->hook >= 2) VectorCopy(routedir, in->wish);
		else IGX_AirWish(pm->vel, routedir, in->wish, x->strafeskill);
		/* pressed while coming down, the jump goes off on the landing frame:
		 * Billy's slingshot (demo, 2026-11-25) lets go in the air and has jump
		 * down before he lands -- off the floor again within 8 ms, 860-940 u/s
		 * kept; a landing without it gives friction the first step */
		in->jump = (o->hop || (o->landhop && (x->released || in->letgo || IGX_PlainLandHop(x, o)) && pm->jumps == x->reljumps)) && pm->vel[2] < 0 && (pm->hook < 2 || in->letgo);
	}
}

/* wish -> forward/side for a view yaw */
static void IGX_Moves(const vec3_t wish, float yaw, float *fm, float *sm)
{
	vec3_t ang, fwd, right;

	VectorSet(ang, 0, yaw, 0);
	AngleVectors(ang, fwd, right, NULL);
	*fm = DotProduct(fwd, wish) * 400;
	*sm = DotProduct(right, wish) * 400;
}

/* ------------------------------------------------------------ simulation */

static void IGX_Start(igx_t *x, igpm_t *pm)
{
	bot_state_t *bs = x->bs;

	IG_PmoveInit(pm, bs->cur_ps.origin, bs->cur_ps.velocity, bs->cur_ps.groundEntityNum != ENTITYNUM_NONE,
				 bs->entitynum, 0);
	if (x->hook == 2) IG_PmoveHookAt(pm, 2, x->hookpos, NULL);
	else if (x->hook == 1) IG_PmoveHookAt(pm, 1, x->hookpos, x->hookdir);
	pm->area = AAS_PointAreaNum(pm->org);
}

static int IGX_EndUnsafe(igx_t *x, igpm_t *pm);

static void IGX_Simulate(igx_t *x, igopt_t *o)
{
	igpm_t pm;
	igin_t in;
	vec3_t route, lastroute, end, vh;
	int t, latch = -1, airjump = -1, half, area, tt;
	float fm, sm, up, wallpen = 0, along, before, after, flow = 0;
	int stop = 0, flown = 0;

	IGX_Start(x, &pm);
	o->ok = 0;
	x->released = !pm.hook;
	x->reljumps = 0;
	if (!IGX_RouteDir(x, pm.org, pm.area, lastroute)) {
		AngleVectors(x->bs->viewangles, lastroute, NULL, NULL);
		lastroute[2] = 0;
		VectorNormalize(lastroute);
	}
	for (t = 0; t < PLAN_THINKS; t++) {
		int special;

		if (pm.hook >= 2 && latch < 0) latch = t;
		if (IGX_RouteDirS(x, pm.org, pm.area, route, &special)) VectorCopy(route, lastroute);
		else VectorCopy(lastroute, route);
		/* on the ground where the route takes a step of its own: the stock
		 * movement takes it from here, and the travel time counts it */
		if (special && t > 0 && pm.ground && !pm.hook) { stop = 1; break; }
		if (!x->released && !pm.hook && latch >= 0) { x->released = 1; x->reljumps = pm.jumps; }	/* lost, or let go */
		IGX_Input(x, o, t, &pm, route, latch, &in);
		if (in.letgo) {
			IG_PmoveHookRelease(&pm);
			if (!x->released) { x->released = 1; x->reljumps = pm.jumps; }
		}
		if (in.fire) {
			IG_PmoveHookFire(&pm, x->lastview[PITCH], x->lastview[YAW]);
			wallpen += x->hookcost * (o->hookmul > 0 ? o->hookmul : 1);
		}
		IGX_Moves(in.wish, in.view[YAW], &fm, &sm);
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		before = VectorLength(vh);
		pm.blocked = 0;
		for (half = 0; half < 2; half++) {
			int pulled = pm.hook >= 2;

			up = in.jump ? 400 : 0;
			/* the game half of strafe chains (ig_botjumpland): a jump pressed
			 * in the air is held two more frames */
			if (x->jumpland) {
				if (pm.ground || pm.hook) airjump = -1;
				else if (up > 0) airjump = t;
				else if (airjump >= 0 && t - airjump <= 2) up = 400;
			}
			IG_Pmove(&pm, fm, sm, up, in.view[PITCH], in.view[YAW], 50);
			/* Billy lets go before the pull drags him onto the floor (5 of 5
			 * in his demo): every 50 ms pulled along the floor costs, as the
			 * friction there takes the pull's speed */
			if (pulled && pm.ground) wallpen += x->pullfloor;
			/* IG-47 (Billy): before a momentum jump the hook is a slingshot -- let
			 * go on this side and make the jump on the speed; still hooked past
			 * the edge is a pull across, predictable: priced ig_fastjumphookacross
			 * (1 s) per 50 ms, with the lip slingshots below the speed comes from */
			if (x->hasjump && pm.hook && !pm.ground) {
				vec3_t pa; VectorSubtract(pm.org, x->jumpfrom, pa); pa[2] = 0;
				if (DotProduct(pa, x->jumpdir) > 0) wallpen += LibVarValue("ig_fastjumphookacross", "1");
			}
		}
		VectorCopy(in.view, x->lastview);	/* the game fires along this next think */
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return;	/* out */
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		after = VectorLength(vh);
		if (pm.blocked && before - after > 200) { wallpen += 0.15f * (before - after) / 300 * LibVarValue("ig_planwallmul", "1") * (x->released ? LibVarValue("ig_planreleasewallmul", "1.5") : 1); if (t < LibVarValue("ig_planwallsoon", "2")) o->wallhit = 1; }
		flow += after;
		flown++;
		if (pm.pad || pm.teleport) break;	/* the navigation file knows where those go */
	}
	if (pm.hook == 1 && pm.hooklost) wallpen += 0.05f;	/* a wasted hook */

	/* Then run the route until the momentum is spent: every option is scored
	 * where its extra speed is used up, not at a fixed time, so speed counts
	 * as the ground it covers. (v1 scored speed at the 1.2 s mark with a
	 * bonus; a slingshot's speed fades after the let-go, so its score fell
	 * every think as the mark slid on, and half the flicks never fired.) */
	if (!pm.pad && !pm.teleport && !stop) {
		igopt_t run;

		Com_Memset(&run, 0, sizeof(run));
		run.anchor = run.fire = -1;
		IG_PmoveHookRelease(&pm);
		for (t = 0; t < PLAN_COAST; t++) {
			int special;

			VectorSet(vh, pm.vel[0], pm.vel[1], 0);
			if (pm.ground && VectorLength(vh) < 330) break;
			if (IGX_RouteDirS(x, pm.org, pm.area, route, &special)) VectorCopy(route, lastroute);
			else VectorCopy(lastroute, route);
			if (special && pm.ground) break;
			IGX_Input(x, &run, PLAN_THINKS + t, &pm, route, -1, &in);
			IGX_Moves(in.wish, in.view[YAW], &fm, &sm);
			before = VectorLength(vh);
			pm.blocked = 0;
			for (half = 0; half < 2; half++)
				IG_Pmove(&pm, fm, sm, 0, in.view[PITCH], in.view[YAW], 50);
			if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return;
			VectorSet(vh, pm.vel[0], pm.vel[1], 0);
			after = VectorLength(vh);
			if (pm.blocked && before - after > 200) { wallpen += 0.15f * (before - after) / 300 * LibVarValue("ig_planwallmul", "1") * (x->released ? LibVarValue("ig_planreleasewallmul", "1.5") : 1); }
			if (pm.pad || pm.teleport) break;
		}
	}

	/* where it ends: the route's time from there */
	area = pm.area;
	VectorCopy(pm.org, end);
	if (!pm.ground && !pm.pad && !pm.teleport && !stop) {	/* still in the air: where it comes down */
		igpm_t fall = pm;

		if (IGX_EndUnsafe(x, &fall)) return;	/* a pit, or lava below */
		VectorCopy(fall.org, end);
		area = fall.area;
	}
	if (!area) return;
	if (area == x->goalarea) tt = 0;
	else if (!(tt = AAS_AreaTravelTimeToGoalArea(area, end, x->goalarea, x->tfl))) return;
	VectorSet(vh, pm.vel[0], pm.vel[1], 0);
	if (!IGX_RouteDir(x, end, area, route)) VectorCopy(lastroute, route);
	along = DotProduct(vh, route);
	o->cost = pm.time + tt * 0.01f - PLAN_SPEEDBONUS * (along > 330 ? (along - 330) / 300 : 0) + wallpen;	/* what speed is left, if the coast ran out */
	/* flow: speed kept is worth something beyond the route time -- harder
	 * to hit, the next slingshot's start (Billy holds 860-930 u/s through
	 * hop after hop). The average over the flight, not a reading at its
	 * end, so the score does not drift as the window slides. */
	if (flown) {
		flow /= flown;
		o->cost -= x->flow * (flow > 330 ? (flow > 930 ? 2.0f : (flow - 330) / 300) : 0);
	}
	o->ok = 1;
}

/* -------------------------------------------------------------- anchors */

/* Anchors come in three classes, priced apart (Billy: "more slingshots and
 * fewer random wall grapples for no reason"):
 *   IGA_SLING  along the route, 6-18 degrees down, 250+ units: the floor or a
 *              far low wall ahead -- his slingshot (median 275 units, 14 down);
 *              price x ig_planslingmul (0.3)
 *   IGA_UP     25 degrees up, only where the route climbs 48+ within 500
 *              units: the stairwell and ledge hook (T6); x ig_planupmul (1)
 *   IGA_SIDE   40 degrees off the route: a turn around a corner, worth it only
 *              when it clearly pays; x ig_plansidemul (2) */
enum { IGA_SLING, IGA_UP, IGA_SIDE, IGA_PADTOP, IGA_CLIMB };
static const char *iga_names[] = { "sling", "up", "side", "padtop", "climb" };

static void IGX_AnchorTry(igx_t *x, float pitch, float yaw, float mind, int cls)
{
	bot_state_t *bs = x->bs;
	vec3_t ang, dir, end, d;
	bsp_trace_t tr;

	if (x->numanchors >= PLAN_MAXANCH) return;
	VectorSet(ang, pitch, yaw, 0);
	AngleVectors(ang, dir, NULL, NULL);
	VectorMA(bs->eye, 900, dir, end);
	BotAI_Trace(&tr, bs->eye, NULL, NULL, end, bs->entitynum, CONTENTS_SOLID | CONTENTS_LAVA | CONTENTS_SLIME);
	if (tr.fraction >= 1 || tr.startsolid) return;
	if (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME)) return;
	if ((tr.surface.flags & SURF_SKY) && !trap_Cvar_VariableIntegerValue("ig_hooksky")) return;
	VectorSubtract(tr.endpos, bs->eye, d);
	if (VectorLength(d) < mind) return;
	/* a slingshot hooks the floor ahead, or a far wall: a wall at a
	 * corridor's end or turn 400 units out was most of q2rdm4's "random
	 * wall grapples" (575 of 898 slingshots there) */
	if (cls == IGA_SLING && tr.plane.normal[2] <= 0.7f && !(x->nearonly && tr.plane.normal[2] < -0.7f) && VectorLength(d) < 600) return;	/* IG-47: a lip ceiling is fine */
	/* IG-47: a momentum jump ahead -- no hook past its take-off: the speed comes
	 * from a slingshot on this side, let go before the edge, then the jump (Billy:
	 * a pull straight across to the far wall is predictable) */
	if (x->hasjump && (x->nearonly || LibVarValue("ig_fastjumpnoacross", "0"))) {
		vec3_t pa; VectorSubtract(tr.endpos, x->jumpfrom, pa); pa[2] = 0;
		if (DotProduct(pa, x->jumpdir) > 0) return;
	}
	VectorCopy(tr.endpos, x->anchors[x->numanchors]);
	vectoangles(d, x->anchorview[x->numanchors]);
	if (x->anchorview[x->numanchors][PITCH] > 180) x->anchorview[x->numanchors][PITCH] -= 360;
	x->anchorclass[x->numanchors] = cls;
	x->anchornz[x->numanchors] = tr.plane.normal[2];
	x->numanchors++;
}

static void IGX_Anchors(igx_t *x, const vec3_t routedir)
{
	/* + is down. From the eye on the ground (46 up) 12 down meets the floor
	 * 220 units out; the bot jumps before it fires, so the hook bites further */
	static const float slingyaw[3] = {-12, 0, 12}, slingpitch[3] = {4, 8, 12};
	float baseyaw = atan2(routedir[1], routedir[0]) * 180 / M_PI;
	int i, j;
	vec3_t ahead;

	x->numanchors = 0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			IGX_AnchorTry(x, slingpitch[j], baseyaw + slingyaw[i], 180, IGA_SLING);
	if (x->goal && trap_BotMovementViewTarget(x->bs->ms, x->goal, x->tfl, 500, ahead) &&
		ahead[2] - x->bs->origin[2] >= 48)
		for (i = -1; i <= 1; i++)
			IGX_AnchorTry(x, -25, baseyaw + 25 * i, 200, IGA_UP);
	for (i = -1; i <= 1; i += 2) {
		IGX_AnchorTry(x, 8, baseyaw + 40 * i, 200, IGA_SIDE);
		IGX_AnchorTry(x, -8, baseyaw + 40 * i, 200, IGA_SIDE);
	}
	/* IG-40 climbs (BOT-TRICKS T9): the climb route's own anchors, hooked from
	 * wherever the bot is -- running, as Billy does -- the sim proves the pull */
	for (i = 0; i < x->nclimb && x->numanchors < PLAN_MAXANCH; i++) {
		vec3_t d, ang;
		int before = x->numanchors;
		VectorSubtract(x->climbanch[i], x->bs->eye, d);
		vectoangles(d, ang);
		IGX_AnchorTry(x, ang[PITCH], ang[YAW], 48, IGA_CLIMB);
		if (x->numanchors > before && Distance(x->anchors[before], x->climbanch[i]) > 32) x->numanchors = before;	/* the hook would bite elsewhere */
	}
	/* IG-47, Billy's slingshot before a momentum jump (ztn2dm3, 5 tries
	 * 2026-11-27): in the air at 110-290 u/s he hooks the floor at the very lip
	 * of the ledge, 120-200 units out and 20-35 degrees down, holds 1-2 frames,
	 * lets go short of the edge at 670-980 u/s and flies the gap. Or the same off
	 * the ceiling above the lip: 10-16 degrees up, 180-270 units, let go at
	 * 935-970 (4 tries). Steep floor and low ceiling anchors short of the edge,
	 * only with a momentum jump ahead (ig_fastjumplip) */
	if (x->hasjump && LibVarValue("ig_fastjumplip", "1")) {
		static const float lippitch[3] = {20, 26, 32}, lipceil[3] = {-10, -14, -18};
		x->nearonly = 1;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				IGX_AnchorTry(x, lippitch[j], baseyaw + slingyaw[i], 100, IGA_SLING);
				IGX_AnchorTry(x, lipceil[j], baseyaw + slingyaw[i], 100, IGA_SLING);
			}
		x->nearonly = 0;
	}
}

/* Billy's pad trick (recorded 2026-11-25, q2rdm11: "a jump pad into a grapple
 * and strafe jumping to get across the map fast"): the vertical pad throws him
 * straight up (718 up, 91 across); rising, he turns 150 degrees to face along
 * the upper walkway; just past the top (falling at 214) he hooks LEVEL and far
 * along it (4 degrees up, a wall ~480 units out), the pull takes him to 938
 * u/s in 0.27 s, he lets go still rising and hops the walkway at 706 u/s, no
 * frame on the ground. The old end-of-arc hook pulled down onto the floor just
 * past the landing and gained nothing. Anchors from the top of a pad flight:
 * along the route past the landing, level to 10 up, 300+ units, walls too. */
static void IGX_PadAnchors(igx_t *x, const vec3_t routedir, const vec3_t toland, int hasroute)
{
	static const float yaws[3] = {-12, 0, 12}, pitches[3] = {2, -3, -8};
	float landyaw = atan2(toland[1], toland[0]) * 180 / M_PI, routeyaw = atan2(routedir[1], routedir[0]) * 180 / M_PI;
	int i, j;

	x->numanchors = 0;
	/* toward the landing (Billy's hook went for the walkway he was flying to) */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			IGX_AnchorTry(x, pitches[j], landyaw + yaws[i], 300, IGA_PADTOP);
	/* and along the route past it */
	if (hasroute)
		for (j = 0; j < 3; j++)
			IGX_AnchorTry(x, pitches[j], routeyaw, 300, IGA_PADTOP);
}

static float IGX_ClassMul(int cls)
{
	if (cls == IGA_PADTOP) return LibVarValue("ig_planpadmul", "0.3");
	if (cls == IGA_CLIMB) return LibVarValue("ig_planclimbmul", "0.2");
	return cls == IGA_SLING ? LibVarValue("ig_planslingmul", "0.3") :
		   cls == IGA_UP ? LibVarValue("ig_planupmul", "1") : LibVarValue("ig_plansidemul", "2");
}

/* ------------------------------------------------------------- the plan */

int IG_PlanOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_plan") != 0;	/* ig-0.5: with or without the hook (no hook: x.canfire 0, runs and hops) */
}

static void IGX_Flick(bot_state_t *bs, const vec3_t view)
{
	int i;

	if (bs->client >= 0 && bs->client < MAX_CLIENTS) {
		igp_state[bs->client].viewset = FloatTime();
		VectorCopy(view, igp_state[bs->client].view);
	}
	VectorCopy(view, bs->ideal_viewangles);
	bs->flags |= BFL_IDEALVIEWSET;
	for (i = 0; i < 2; i++) {
		bs->viewangles[i] = AngleMod(view[i]);
		bs->viewanglespeed[i] = 0;
	}
}

static int		igp_padmode;	/* IG_Plan called by IG_PlanPadTop: a pad flight */
static vec3_t	igp_padland, igp_padtoland;
static int		igp_padroute;

static float IGX_HSpeed(bot_state_t *bs)
{
	return sqrt(bs->cur_ps.velocity[0] * bs->cur_ps.velocity[0] + bs->cur_ps.velocity[1] * bs->cur_ps.velocity[1]);
}

int IG_Plan(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	igx_t x;
	igopt_t opts[PLAN_MAXOPT];
	int n = 0, i, best = -1, pick, c = bs->client, ground, ours, k, groundhop, urge;
	float bestcost = 1e9f, total, r, w[PLAN_MAXOPT];
	vec3_t eye, routedir, hookpos;
	igpm_t pm;
	igin_t in;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || BotIsDead(bs) || !goal || !goal->areanum) return 0;
	igp_lastplan[c] = FloatTime();
	if (FloatTime() - igp_state[c].decided < 0.001f) {		/* decided this think already: the same again */
		if (igp_state[c].setmove) EA_IGMove(c, igp_state[c].wish, 400, igp_state[c].jump, 0);
		if (FloatTime() - igp_state[c].viewset < 0.001f) IGX_Flick(bs, igp_state[c].view);
		return igp_state[c].ret;
	}
	igp_state[c].decided = FloatTime();
	igp_state[c].ret = 0;
	igp_state[c].setmove = 0;
	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	x.goalarea = goal->areanum;
	x.goal = goal;
	VectorCopy(goal->origin, x.goalorg);
	x.tfl = bs->tfl | IG_FastJumpTfl(bs->client, bs->cur_ps.velocity);	/* IG-47 */
	/* IG-47: the planner plans with momentum jumps from ig_fastjumpplan u/s
	 * (250, Billy's ztn2dm3 ledge: 0 = only when the route has them), so it builds the speed toward
	 * one coming up; its simulation proves the jump or rules it out */
	if (LibVarValue("ig_fastjumps", "0") && LibVarValue("ig_fastjumpplan", "250") > 0 &&
		sqrt(bs->cur_ps.velocity[0] * bs->cur_ps.velocity[0] + bs->cur_ps.velocity[1] * bs->cur_ps.velocity[1]) >= LibVarValue("ig_fastjumpplan", "250"))
		x.tfl |= TFL_STRAFEJUMP;
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2) {
		vec3_t o2; int a2;
		VectorCopy(bs->cur_ps.origin, o2); a2 = AAS_PointAreaNum(o2);
		BotAI_Print(PRT_MESSAGE, "igfj %d tfl %x fast %d area %d goal %d tt %d ttfast %d v %.0f\n", bs->client, x.tfl, (x.tfl & TFL_STRAFEJUMP) != 0, a2, x.goalarea,
			a2 ? AAS_AreaTravelTimeToGoalArea(a2, o2, x.goalarea, bs->tfl) : -1, a2 ? AAS_AreaTravelTimeToGoalArea(a2, o2, x.goalarea, bs->tfl | TFL_STRAFEJUMP) : -1, VectorLength(bs->cur_ps.velocity));
	}
	x.jumpland = trap_Cvar_VariableIntegerValue("ig_jumpland");
	x.canfire = IG_CanFireHook(c) && LibVarValue("bot_grapple", "0");	/* no hook in the mode (bot_grapple 0): runs and hops only */
	x.flow = LibVarValue("ig_planflow", "0.2");
	x.strafeskill = IG_StrafeSkill(bs);
	{	/* once per bot and map: its strafe skill, for the logs */
		static int shown[MAX_CLIENTS];
		static float lastt;
		int k2;
		char name[MAX_NETNAME];

		if (FloatTime() < lastt) for (k2 = 0; k2 < MAX_CLIENTS; k2++) shown[k2] = 0;
		lastt = FloatTime();
		if (!shown[c]) {
			shown[c] = 1;
			ClientName(c, name, sizeof(name));
			BotAI_Print(PRT_MESSAGE, "ig: strafeskill %d %.2f %s\n", c, x.strafeskill, name);
		}
	}
	urge = IG_PlanRepositionUrge(c);
	x.pullfloor = LibVarValue("ig_planpullfloor", "0.25");
	/* a slingshot is flown with Billy's finish -- let go before the floor,
	 * hop the landing -- so the anchors whose line has room to hop score best */
	x.slinghop = LibVarValue("ig_planslinghop", "1") != 0;
	x.hookcost = LibVarValue("ig_planhookcost", "0.5") * (1.6f - IG_PlanAffinity(bs)) / (1.6f - 0.55f);
	/* Billy, watching his q2rdm11 pad route drilled: "I was hoping to have just
	 * the one grapple and strafe jumping used to keep momentum". Carrying speed,
	 * a hook must gain a lot to beat hopping on: above 450 u/s across its price
	 * grows, x2.7 at 700, x4 at 900 (ig_planfasthook 0: off). Not in a pad's
	 * flight, which starts slow. */
	if (!igp_padmode && LibVarValue("ig_planfasthook", "1")) {
		float hs = IGX_HSpeed(bs);

		if (hs > 450) x.hookcost *= 1 + (hs - 450) / LibVarValue("ig_planfasthookdiv", "150");
	}
	VectorCopy(bs->viewangles, x.lastview);
	if (x.lastview[PITCH] > 180) x.lastview[PITCH] -= 360;
	if (igp_state[c].flicktime > 0 && FloatTime() - igp_state[c].flicktime < 0.15f && FloatTime() - igp_state[c].flicktime > 0.001f)
		VectorCopy(igp_state[c].flickview, x.lastview);

	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	x.hook = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), hookpos);
	VectorCopy(hookpos, x.hookpos);
	if (x.hook == 1) {
		if (igp_state[c].hookout) VectorCopy(igp_state[c].firedir, x.hookdir);	/* ours: we know where it went */
		else {
			VectorSubtract(hookpos, eye, x.hookdir);
			VectorNormalize(x.hookdir);
		}
	}
	ground = bs->cur_ps.groundEntityNum != ENTITYNUM_NONE;
	ours = igp_state[c].active || (x.hook && igp_state[c].hookout);
	if (!x.hook) igp_state[c].hookout = 0;
	if (ground && !x.hook) igp_state[c].active = 0;
	/* landed since the last think with a landing hop pending: hop now, at
	 * most one 50 ms step late (a let-go just above the floor lands inside
	 * the let-go's own think) -- then it is done */
	groundhop = ground && igp_state[c].landhop && !x.hook;
	if (ground) igp_state[c].landhop = 0;
	if (ground) igp_state[c].padfired = 0;

	/* IG-40 climbs: a grapple route within reach on the route, climbed by the planner */
	IGX_FindClimb(&x, bs->origin);
	/* what the planner does not plan: the stock movement's */
	if (!igp_padmode && !ours && (((mr->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK &&
								   !(x.climbrn && (mr->traveltype & TRAVELTYPE_MASK) == TRAVEL_GRAPPLEHOOK)) ||
				  (mr->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE)))) {
		if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2)
			BotAI_Print(PRT_MESSAGE, "igret %d stock travel %d flags %d ground %d speed %.0f\n", c, mr->traveltype & TRAVELTYPE_MASK, mr->flags, ground, IGX_HSpeed(bs));
		return 0;
	}
	if (x.hook && !igp_state[c].hookout) {		/* a climb route's hook: not ours */
		if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2 && igp_state[c].landhop)
			BotAI_Print(PRT_MESSAGE, "iglh %d stepped back: a hook not ours, sticky %d\n", c, igp_state[c].landhop);
		return 0;
	}
	if (AAS_PointContents(bs->origin) & (CONTENTS_WATER | CONTENTS_LAVA | CONTENTS_SLIME)) return 0;

	IGX_Start(&x, &pm);
	if (igp_padmode) {		/* a pad flight: the route from where the pad lands it */
		vec3_t up;
		int la;

		VectorCopy(igp_padland, up);
		up[2] += 16;
		la = AAS_PointAreaNum(up);
		VectorSubtract(igp_padland, bs->origin, igp_padtoland);
		igp_padtoland[2] = 0;
		VectorNormalize(igp_padtoland);
		igp_padroute = la && IGX_RouteDir(&x, up, la, routedir);
		if (!igp_padroute) {
			/* the landing's own area has no step (it is the goal's, say): toward the goal, else the landing */
			VectorSubtract(goal->origin, igp_padland, routedir);
			routedir[2] = 0;
			if (VectorNormalize(routedir) < 64) VectorCopy(igp_padtoland, routedir);
			else igp_padroute = 1;
		}
	} else if ((IGX_LookAhead(&x, pm.org, pm.area), IGX_JumpKeep(&x), !IGX_RouteDir(&x, pm.org, pm.area, routedir))) {
		if (!ours) return 0;
		VectorSet(routedir, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		if (VectorNormalize(routedir) < 1) return 0;
	}

	/* the options from here */
#define OPT(K, H, J0, A, F, R, L) do { if (n < PLAN_MAXOPT) { igopt_t *o_ = &opts[n++]; o_->kind = K; o_->hop = H; o_->hookmul = 1; o_->landhop = 0; o_->wallhit = 0; \
	o_->jump0 = J0; o_->anchor = A; o_->fire = F; o_->relafter = R; o_->letgo = L; } } while (0)
	if (x.hook) {
		OPT(PO_LETGO, 0, 0, -1, -1, 0, 1);
		{ OPT(PO_LANDHOP, 0, 0, -1, -1, 0, 1); opts[n - 1].landhop = 1; }	/* let go, hop the landing */
		OPT(PO_LETGOHOP, 1, 0, -1, -1, 0, 1);
		for (k = 1; k <= 4; k++) OPT(PO_HOLD, 0, 0, -1, -1, k * k == 16 ? 8 : k, 0);	/* 1, 2, 3, 8 thinks */
		/* T3, the pull hop: hop off the floor while the hook drags (friction eats the pull) */
		if (ground) for (k = 2; k <= 3; k++) OPT(PO_HOLDHOP, 1, 0, -1, -1, k, 0);
	} else {
		/* Billy: "the bots sometimes just jump in place ... the jumping should be
		 * in use for movement purposes". A hop from a standstill gains nothing (the
		 * ground accelerates faster than the air), yet scored near the run and was
		 * picked now and then -- after a respawn, say. On the ground, hops only
		 * when already moving (ig_planhopmin u/s across, default 250). */
		int hopok = !igp_padmode && (!ground || IGX_HSpeed(bs) >= LibVarValue("ig_planhopmin", "250"));

		OPT(PO_RUN, 0, 0, -1, -1, 0, 0);
		if (hopok) OPT(PO_HOP, 1, ground, -1, -1, 0, 0);
		if ((!ground || groundhop) && (hopok || (igp_padmode && igp_state[c].padfired))) { OPT(PO_LANDHOP, 0, 0, -1, -1, 0, 0); opts[n - 1].landhop = 1; }	/* hop the (next) landing; after a pad-top slingshot too */
		/* flicked onto an anchor last think: it can go now */
		if (igp_state[c].anchorset && FloatTime() - igp_state[c].flicktime < 0.15f &&
			FloatTime() - igp_state[c].flicktime > 0.001f && x.canfire) {
			{ OPT(PO_FIRE, 0, 0, -1, 0, 3, 0); opts[n - 1].hookmul = IGX_ClassMul(igp_state[c].anchorclass); opts[n - 1].landhop = x.slinghop; }
		}
		if (igp_padmode) {
			/* the top of the arc, once a flight (Billy fired falling at 214) */
			if (x.canfire && !igp_state[c].padfired && bs->cur_ps.velocity[2] < LibVarValue("ig_planpadtopvz", "150")) {
				IGX_PadAnchors(&x, routedir, igp_padtoland, igp_padroute);
				for (i = 0; i < x.numanchors; i++)
					{ OPT(PO_SLING, 0, 0, i, 1, 3, 0); opts[n - 1].hookmul = IGX_ClassMul(x.anchorclass[i]); opts[n - 1].landhop = x.slinghop; }
			}
		} else if (x.canfire) {
			IGX_Anchors(&x, routedir);
			for (i = 0; i < x.numanchors; i++)
				{ OPT(PO_SLING, 0, ground, i, 1, 3, 0); opts[n - 1].hookmul = IGX_ClassMul(x.anchorclass[i]); opts[n - 1].landhop = x.slinghop; }
		}
	}
#undef OPT

	for (i = 0; i < n; i++) {
		vec3_t keepview;

		VectorCopy(x.lastview, keepview);
		IGX_Simulate(&x, &opts[i]);
		VectorCopy(keepview, x.lastview);
		if (!opts[i].ok) continue;
		/* the option under way gets the benefit of the doubt */
		/* a bot that snapped its view onto a spot hooks it: flicks that came
		 * to nothing were half of them in v1, a view twitching at walls */
		/* a reposition urged (a kill, two misses): hook away now (IG-10's
		 * travelling half, which the old slingshot step did and the planner
		 * had dropped -- the urge waited for the next fight instead) */
		if (urge && (opts[i].kind == PO_SLING || opts[i].kind == PO_FIRE)) opts[i].cost -= 0.5f;
		if (opts[i].kind == PO_FIRE) opts[i].cost -= PLAN_COMMIT;
		/* a landing hop decided (every let-go decides one) is a promise too,
		 * kept through the simulation: forced presses instead doubled the
		 * wall hits and raised the lava trips */
		else if (opts[i].kind == PO_LANDHOP && (igp_state[c].landhop || groundhop)) opts[i].cost -= PLAN_COMMIT;
		/* in the air with speed, the landing is hopped as a habit (Billy hops
		 * every one; the planner's "run" handed the flight to the stock move,
		 * which never hops: 808 -> 302 u/s on the walkway). ig_planlandhopbonus */
		else if (opts[i].kind == PO_LANDHOP && !ground && IGX_HSpeed(bs) >= 400) opts[i].cost -= LibVarValue("ig_planlandhopbonus", "0.25");
		else if (opts[i].kind == igp_state[c].kind ||
				 (opts[i].kind == PO_HOLDHOP && igp_state[c].kind == PO_HOLD) || (opts[i].kind == PO_HOLD && igp_state[c].kind == PO_HOLDHOP))
			opts[i].cost -= PLAN_KEEP;
		if (opts[i].cost < bestcost) {
			bestcost = opts[i].cost;
			best = i;
		}
	}
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2 && igp_state[c].anchorset && FloatTime() - igp_state[c].flicktime < 0.15f) {
		int nf = 0, fok = 0;
		float fc = 0;

		for (i = 0; i < n; i++)
			if (opts[i].kind == PO_FIRE) { nf++; fok = opts[i].ok; fc = opts[i].cost; }
		BotAI_Print(PRT_MESSAGE, "igpafter %d fireopt %d ok %d cost %.2f best %.2f canfire %d hook %d dt %.3f ground %d\n", c, nf, fok, fc,
					bestcost, x.canfire, x.hook, FloatTime() - igp_state[c].flicktime, ground);
	}
	if (igp_padmode && trap_Cvar_VariableIntegerValue("ig_plandebug") == 3) {
		int ns = 0, nok = 0;
		float runc = -1, bests = 1e9f;

		for (i = 0; i < n; i++) {
			if (opts[i].kind == PO_RUN && opts[i].ok) runc = opts[i].cost;
			if (opts[i].kind == PO_SLING) { ns++; if (opts[i].ok) { nok++; if (opts[i].cost < bests) bests = opts[i].cost; } }
		}
		BotAI_Print(PRT_MESSAGE, "igpad %d vz %.0f anchors %d slings %d ok %d run %.2f bestsling %.2f pick %s fired %d canfire %d\n", c,
					bs->cur_ps.velocity[2], x.numanchors, ns, nok, runc, bests, best >= 0 ? po_names[opts[best].kind] : "-", igp_state[c].padfired, x.canfire);
	}
	if (best < 0) {
		/* every option ends in lava or a pit: let go if hooked, else the stock move */
		if (x.hook) trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
		igp_state[c].active = 0;
		return x.hook != 0;
	}
	/* a fair pick among the near-best: hooks weighted by the bot's affinity */
	total = 0;
	for (i = 0; i < n; i++) {
		w[i] = 0;
		if (!opts[i].ok || opts[i].cost > bestcost + PLAN_MARGIN) continue;
		/* ig_planhookpick: how much a hook is favoured among near-equal options (1 = not at all) */
		w[i] = opts[i].kind == PO_SLING || opts[i].kind == PO_FIRE ? 1 + (IG_PlanAffinity(bs) * 2 - 1) * LibVarValue("ig_planhookpick", "1") : 1;
		total += w[i];
	}
	r = random() * total;
	for (pick = best, i = 0; i < n; i++) {
		if (w[i] <= 0) continue;
		if (r < w[i]) { pick = i; break; }
		r -= w[i];
	}
	/* In the air with speed, the landing is hopped as a habit -- unless that is
	 * clearly worse (lava, a pit, a wall: ruled out or priced by the
	 * simulation). Roaming, 4 maps: 48% of fast landings were hopped (Billy
	 * nearly all); 500 of the flat ones were "run" (or "let go") picked
	 * by chance over a near-equal "hop the landing". ig_planairhop */
	/* ... and on the think after a landing with the hop still pending: Quake II
	 * checks the jump at the start of each step, so a press that came in the
	 * step that landed did not count, and "run" here went flat (262 of 4 maps'
	 * flat landings had jump down). */
	if ((!ground || groundhop) && IGX_HSpeed(bs) >= LibVarValue("ig_planairhopspeed", "300") && (opts[pick].kind == PO_RUN || opts[pick].kind == PO_LETGO) &&
		x.turnahead < LibVarValue("ig_planturnhop", "100") &&		/* a sharp corner within the hop: land and turn (Billy) */
		LibVarValue("ig_planairhop", "1")) {
		int want_letgo = opts[pick].kind == PO_LETGO;

		for (i = 0; i < n; i++)
			if (opts[i].kind == PO_LANDHOP && opts[i].ok && !opts[i].wallhit && (opts[i].letgo != 0) == want_letgo &&
				opts[i].cost <= opts[pick].cost + LibVarValue("ig_planairhopmargin", "0.5")) { pick = i; break; }
	}

	/* IG-40 pad flights (2026-11-28): in a route's pad flight the planner flies
	 * only its own hook -- the pad-top slingshot fired or under way. Picking
	 * "run" or a hop it took over the air steering anyway, along the route
	 * beyond the landing and for speed, and pushed bots off q2rdm11's twin-pad
	 * ledge (11 of 11 hook-free misses in 16 fight matches were these); the
	 * stock pad air control, which lands them, keeps the flight. */
	if (igp_padmode && !x.hook && opts[pick].kind != PO_SLING && opts[pick].kind != PO_FIRE && LibVarValue("ig_planpadkeep", "1")) {
		igp_state[c].active = 0;
		return 0;
	}
	/* do its first think */
	{
		igopt_t *o = &opts[pick];

		IGX_Input(&x, o, 0, &pm, routedir, x.hook == 2 ? 0 : -1, &in);
		if (in.letgo && x.hook) {
			trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
			igp_state[c].hookout = 0;
		}
		if (o->kind == PO_RUN) {
			igp_state[c].kind = PO_RUN;
			igp_state[c].anchorset = 0;
			if (!ground && igp_state[c].active) {	/* our flight: steer it */
				EA_IGMove(c, in.wish, 400, 0, 0);
				igp_state[c].steered = FloatTime();
				igp_state[c].setmove = 1;
				VectorCopy(in.wish, igp_state[c].wish);
				igp_state[c].jump = 0;
				return igp_state[c].ret = 1;
			}
			igp_state[c].active = 0;
			return 0;	/* the stock route move */
		}
		/* a landing hop is a decision: jump stays down on the way down until
		 * it lands (decided while rising, the hop was lost whenever a later
		 * think picked another option: 13% of landings hopped, Billy 100%) */
		if (o->kind == PO_LANDHOP) igp_state[c].landhop = 1;
		/* every let-go hops its landing, as Billy's all do, unless that
		 * landing hop was ruled out (lava, a pit) */
		if (in.letgo) {
			int lok = -1;

			for (i = 0; i < n; i++)
				if (opts[i].kind == PO_LANDHOP) { lok = opts[i].ok; if (opts[i].ok) igp_state[c].landhop = 1; }
			if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2)
				BotAI_Print(PRT_MESSAGE, "iglh %d letgo kind %s landhop-option %d sticky %d\n", c, po_names[o->kind], lok, igp_state[c].landhop);
		}
		if (o->kind == PO_SLING || o->kind == PO_FIRE) igp_state[c].landhop = 0;
		/* still rising, but down within this think (a step, stairs, a ledge):
		 * the jump goes down for its second half (a delayed jump), or the
		 * landing comes before the next think and goes flat. ig_planlatejump */
		if (!in.jump && o->landhop && !ground && pm.vel[2] >= 0 && LibVarValue("ig_planlatejump", "1")) {
			igpm_t p2 = pm;
			float fm, sm;
			int h;

			IGX_Moves(in.wish, x.lastview[YAW], &fm, &sm);
			for (h = 0; h < 2 && !p2.ground; h++)
				IG_Pmove(&p2, fm, sm, 0, x.lastview[PITCH], x.lastview[YAW], 50);
			if (p2.ground) in.jump = 2;
		}
		EA_IGMove(c, in.wish, 400, in.jump, 0);
		igp_state[c].steered = FloatTime();
		igp_state[c].setmove = 1;
		if (x.climbrn) IG_PlanClimbNote(c, x.climbrn);	/* IG-48: the stock grapple stands down next think only because the planner moved now */
		VectorCopy(in.wish, igp_state[c].wish);
		igp_state[c].jump = in.jump;
		if (o->anchor >= 0) {
			IGX_Flick(bs, x.anchorview[o->anchor]);
			VectorCopy(x.anchorview[o->anchor], igp_state[c].flickview);
			VectorCopy(x.anchors[o->anchor], igp_state[c].anchor);
			igp_state[c].anchorclass = x.anchorclass[o->anchor];
			igp_state[c].anchornz = x.anchornz[o->anchor];
			igp_state[c].flicktime = FloatTime();
			igp_state[c].anchorset = 1;
		} else if (o->kind == PO_FIRE) {
			IGX_Flick(bs, igp_state[c].flickview);	/* hold the view the hook goes along */
		}
		if (in.fire) {
			vec3_t d;

			trap_EA_Command(c, LibVarGetString("cmd_grappleon"));
			IG_NoteHookFire(c);
			if (igp_state[c].anchorclass == IGA_CLIMB) igpc_fired[c] = 1;	/* IG-40 climbs */
			IG_GrappleTravelNote(c);
			igp_state[c].hookout = 1;
			if (igp_padmode) { igp_state[c].padfired = 1; IG_PadDrillNoteTop(c); }
			igp_state[c].anchorset = 0;
			AngleVectors(x.lastview, igp_state[c].firedir, NULL, NULL);
			VectorSubtract(igp_state[c].anchor, eye, d);
			BotAI_Print(PRT_MESSAGE, "ig: slingshot %d length %.0f chain %d air %d class %s %s\n", c, VectorLength(d),
						igp_state[c].chain++, !ground, iga_names[igp_state[c].anchorclass],
						igp_state[c].anchornz > 0.7f ? "floor" : igp_state[c].anchornz < -0.7f ? "ceiling" : "wall");
			if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2)
				BotAI_Print(PRT_MESSAGE, "igfire %d view %.1f %.1f anchor %.0f %.0f %.0f eye %.0f %.0f %.0f\n", c,
							x.lastview[PITCH], x.lastview[YAW], igp_state[c].anchor[0], igp_state[c].anchor[1],
							igp_state[c].anchor[2], eye[0], eye[1], eye[2]);
		}
		if (x.hook) IG_GrappleTravelNote(c);	/* ours: the watchdog keeps off */
		if (ground && o->kind != PO_HOLD && o->kind != PO_HOLDHOP) igp_state[c].chain = 0;
		igp_state[c].kind = o->kind;
		igp_state[c].active = 1;
		if (trap_Cvar_VariableIntegerValue("ig_plandebug") && ((o->kind != PO_HOLD && o->kind != PO_HOLDHOP) || FloatTime() - igp_state[c].lastlog > 0.5f)) {
			igp_state[c].lastlog = FloatTime();
			BotAI_Print(PRT_MESSAGE, "ig: plan %d %s cost %.2f best %.2f options %d\n", c, po_names[o->kind], o->cost, bestcost, n);
		}
	}
	return igp_state[c].ret = 1;
}

/* ---------------------------------------------------------------- guard */

/* Does holding this input for GUARD_THINKS end in lava, slime or a pit?
 * The body as it is now, the bot's own hook included. */
#define GUARD_THINKS	5
static int IGX_Unsafe(igx_t *x, const vec3_t dir, float speed, int jump, int crouch, float yaw, float pitch)
{
	igpm_t pm;
	vec3_t wish;
	float fm, sm;
	int t, half;

	IGX_Start(x, &pm);
	VectorSet(wish, dir[0], dir[1], 0);
	if (VectorNormalize(wish) < 0.01f) speed = 0;
	IGX_Moves(wish, yaw, &fm, &sm);
	fm *= speed / 400;
	sm *= speed / 400;
	for (t = 0; t < GUARD_THINKS; t++) {
		for (half = 0; half < 2; half++)
			IG_Pmove(&pm, fm, sm, jump && t == 0 ? 400 : crouch ? -400 : 0, pitch, yaw, 50);
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return 1;
		if (pm.pad || pm.teleport) return 0;
	}
	return IGX_EndUnsafe(x, &pm);
}

/* Every think, after the AI has made its input (be_interface_q2.c): an input
 * the planner did not make itself -- a fight's dodges, the stock route move --
 * that would take the bot into lava or a pit within half a second is changed
 * for the nearest safe one: the same without the jump, then turned 45, 90 and
 * 135 degrees either way, then still. Combat strafing walked bots off
 * q2rdm11's walkways into the lava; stopping or reversing outright, the first
 * version of this, cost brutalo's platform fights 40 u/s. */
static void IGX_InputCheck(bot_state_t *bs, const bot_input_t *bi)
{
	int c = bs->client, jump = (bi->actionflags & (ACTION_JUMP | ACTION_DELAYEDJUMP)) != 0;
	vec3_t d;

	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 4 && bs->cur_ps.groundEntityNum == ENTITYNUM_NONE) {
		vec3_t vh, dh;
		float sp = 0, ang = -1;

		VectorSet(vh, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		VectorSet(dh, bi->dir[0], bi->dir[1], 0);
		sp = VectorNormalize(vh);
		if (VectorNormalize(dh) > 0.01f && sp > 1 && bi->speed > 0) ang = RAD2DEG(acos(DotProduct(vh, dh) > 1 ? 1 : DotProduct(vh, dh)));
		BotAI_Print(PRT_MESSAGE, "igair %d steered %d node %s speed %.0f input %.0f angle %.0f\n", c,
					FloatTime() - igp_state[c].steered < 0.001f, igp_node[c], sp, bi->speed, ang);
	}
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") != 2 || FloatTime() - igp_state[c].steered > 0.001f) return;
	VectorSet(d, bi->dir[0], bi->dir[1], 0);
	VectorNormalize(d);
	BotAI_Print(PRT_MESSAGE, "igpin %d dirdot %.2f speed %.0f jump %d/%d keys %d view %.1f/%.1f %.1f/%.1f\n", c,
				DotProduct(d, igp_state[c].wish), bi->speed, jump, igp_state[c].jump,
				(bi->actionflags & (ACTION_MOVEFORWARD | ACTION_MOVEBACK | ACTION_MOVELEFT | ACTION_MOVERIGHT | ACTION_CROUCH)) != 0,
				bi->viewangles[PITCH], bi->viewangles[YAW], igp_state[c].view[PITCH], igp_state[c].view[YAW]);
}

/* No jump from (near) a standstill (Billy: "the bots sometimes just jump in
 * place when they dont have any movement ... the jumping should be in use for
 * movement purposes"). On the final input: a jump the planner or the fight
 * planner set this think, or the stock fight move's (its jumper trait), with
 * under ig_planhopmin u/s across (default 250; 0 = off) goes. Kept: the
 * slingshot's start (jump, then fire) and the stock route's own jumps (a
 * ledge, a gap): those have a purpose. 8 maps x 2: jumps from under 100 u/s
 * were 1544, of which landing hops after a slow landing were the most. */
static void IGX_NoStandingJump(bot_state_t *bs, bot_input_t *bi)
{
	int c = bs->client;
	float min = LibVarValue("ig_planhopmin", "250");

	if (min <= 0 || !(bi->actionflags & (ACTION_JUMP | ACTION_DELAYEDJUMP)) || IGX_HSpeed(bs) >= min) return;
	if (FloatTime() - igp_state[c].steered < 0.001f) {
		if (FloatTime() - igp_state[c].decided < 0.001f &&
			(igp_state[c].kind == PO_SLING || igp_state[c].kind == PO_FIRE)) return;	/* a slingshot's start */
	} else if (strcmp(igp_node[c], "battle fight")) return;		/* the route's own */
	bi->actionflags &= ~(ACTION_JUMP | ACTION_DELAYEDJUMP);
	igp_state[c].landhop = 0;
}

void IG_PlanGuard(bot_state_t *bs, bot_input_t *bi)
{
	if (bs->client >= 0 && bs->client < MAX_CLIENTS) IGX_InputCheck(bs, bi);	/* diagnostics */
	if (IG_PlanOn() && bs->client >= 0 && bs->client < MAX_CLIENTS && !BotIsDead(bs)) IGX_NoStandingJump(bs, bi);
	igx_t x;
	vec3_t eye, back, zero = {0, 0, 0};
	int c = bs->client, jump, crouch;
	float pitch;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || BotIsDead(bs)) return;
	if (!LibVarValue("ig_planguard", "1")) return;	/* testing: the guard off */
	if (FloatTime() - igp_state[c].steered < 0.001f) return;	/* the planner's own, already flown */
	/* the route's own move follows the navigation file, which keeps off lava;
	 * fights dodge with no thought for edges */
	if (strncmp(igp_node[c], "battle", 6)) return;
	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	x.hook = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), x.hookpos);
	if (x.hook == 1) {
		VectorSubtract(x.hookpos, eye, x.hookdir);
		VectorNormalize(x.hookdir);
	}
	jump = (bi->actionflags & (ACTION_JUMP | ACTION_DELAYEDJUMP)) != 0;
	crouch = (bi->actionflags & ACTION_CROUCH) != 0;
	pitch = bi->viewangles[PITCH] > 180 ? bi->viewangles[PITCH] - 360 : bi->viewangles[PITCH];
	if (bi->actionflags & (ACTION_MOVEFORWARD | ACTION_MOVEBACK | ACTION_MOVELEFT | ACTION_MOVERIGHT)) return;	/* key moves: rare, left alone */
	if (!IGX_Unsafe(&x, bi->dir, bi->speed, jump, crouch, bi->viewangles[YAW], pitch)) return;
	if (jump && !IGX_Unsafe(&x, bi->dir, bi->speed, 0, crouch, bi->viewangles[YAW], pitch)) {
		bi->actionflags &= ~(ACTION_JUMP | ACTION_DELAYEDJUMP);
		BotAI_Print(PRT_MESSAGE, "ig: guard %d nojump node %s\n", c, igp_node[c]);
		return;
	}
	{
		static const float turns[6] = {45, -45, 90, -90, 135, -135};
		int i, first = random() < 0.5f;		/* either side first, by chance */

		for (i = 0; i < 6; i++) {
			float a = DEG2RAD(turns[i ^ (first && i < 6 ? 1 : 0)]);

			back[0] = bi->dir[0] * cos(a) - bi->dir[1] * sin(a);
			back[1] = bi->dir[0] * sin(a) + bi->dir[1] * cos(a);
			back[2] = 0;
			if (!IGX_Unsafe(&x, back, bi->speed, 0, crouch, bi->viewangles[YAW], pitch)) {
				VectorCopy(back, bi->dir);
				bi->actionflags &= ~(ACTION_JUMP | ACTION_DELAYEDJUMP);
				BotAI_Print(PRT_MESSAGE, "ig: guard %d turn%d node %s\n", c, (int)fabs(turns[i]), igp_node[c]);
				return;
			}
		}
	}
	if (!IGX_Unsafe(&x, zero, 0, 0, crouch, bi->viewangles[YAW], pitch)) {
		bi->speed = 0;
		bi->actionflags &= ~(ACTION_JUMP | ACTION_DELAYEDJUMP);
		BotAI_Print(PRT_MESSAGE, "ig: guard %d stop node %s\n", c, igp_node[c]);
	} else {
		BotAI_Print(PRT_MESSAGE, "ig: guard %d none node %s\n", c, igp_node[c]);
	}
}

/* ------------------------------------------------- the fight hook's checks */

/* Where a body that is not (any more) on the ground comes down: a pit or
 * lava below is unsafe. */
static int IGX_EndUnsafe(igx_t *x, igpm_t *pm)
{
	int t;

	(void)x;
	/* fly the fall out, up to 2.5 s: a pit's death zone is lava in the
	 * navigation file (IGPM_Categorize), and it lies near the bottom */
	IG_PmoveHookRelease(pm);
	for (t = 0; t < 50 && !pm->ground && !pm->pad && !pm->teleport; t++) {
		IG_Pmove(pm, 0, 0, 0, 0, 0, 50);
		if (pm->everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return 1;
	}
	return !pm->ground && !pm->pad && !pm->teleport;
}

/* The hook out now (fire: or fired now at `anchor` along the flicked view)
 * held `hold` thinks after it bites, then let go, and the body left to fly:
 * does it end in lava, slime or a pit within PLAN_THINKS? */
static int IGX_HookUnsafe(bot_state_t *bs, const vec3_t anchor, int hold)
{
	igx_t x;
	igpm_t pm;
	vec3_t eye, d, view;
	int t, half, latch = -1;

	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	if (!anchor) {
		x.hook = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), x.hookpos);
		if (!x.hook) return 0;
		if (x.hook == 1) {
			VectorSubtract(x.hookpos, eye, x.hookdir);
			VectorNormalize(x.hookdir);
		}
	}
	IGX_Start(&x, &pm);
	if (anchor) {
		VectorSubtract(anchor, eye, d);
		vectoangles(d, view);
		if (view[PITCH] > 180) view[PITCH] -= 360;
		IG_PmoveHookFire(&pm, view[PITCH], view[YAW]);
	}
	for (t = 0; t < PLAN_THINKS; t++) {
		if (pm.hook >= 2 && latch < 0) latch = t;
		if (latch >= 0 && t - latch >= hold) IG_PmoveHookRelease(&pm);
		for (half = 0; half < 2; half++)
			IG_Pmove(&pm, 0, 0, 0, 0, 0, 50);
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return 1;
		if (pm.pad || pm.teleport) return 0;
	}
	return IGX_EndUnsafe(&x, &pm);
}

int IG_PlanFireUnsafe(bot_state_t *bs, const vec3_t anchor, int hold)
{
	return IGX_HookUnsafe(bs, anchor, hold);
}

int IG_PlanHookUnsafe(bot_state_t *bs, int hold)
{
	return IGX_HookUnsafe(bs, NULL, hold);
}

/* ------------------------------------------------------------- fights */

/* The fight movement (IG-40, in place of BotAttackMove). A railgun fight is
 * one shot each way, so a dodge is good when it is hard to hit -- fast
 * across the enemy's line -- and when it PEEKS: in sight of the enemy while
 * the rail is loaded, out of it while it reloads (Billy: quick-peeking a
 * corner). Each think, FIGHT_OPTS dodges (strafe either side, angled in, out
 * or square, with and without a hop; straight in; straight out) are flown
 * FIGHT_THINKS in Quake II's movement against the enemy where it is now and
 * costed; lava, slime and pits are out, as everywhere. A dodge is kept for a
 * random 0.3-0.8 s unless it turns bad, so the bot does not jitter. */
#define FIGHT_THINKS	8
#define FIGHT_RELOAD	1.5f	/* the railgun's cycle, s (libvar ig_planreload) */

static struct {
	int		opt;			/* the dodge under way, -1 none */
	float	until;			/* kept until then */
	float	lastshot;
	int		shooting;
	float	range;			/* the range this fight wants (ig_pfstance) */
	float	rangeuntil;
	int		rangeenemy;
} igf[MAX_CLIENTS];

typedef struct {
	vec3_t	dir;
	int		hop;
	float	cost;
	int		ok;
} igfopt_t;

static int IGX_Sees(bot_state_t *bs, const vec3_t eye, const vec3_t target)
{
	bsp_trace_t tr;
	vec3_t a, b;

	VectorCopy(eye, a);
	VectorCopy(target, b);
	BotAI_Trace(&tr, a, NULL, NULL, b, bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP);
	return tr.fraction >= 1;
}

static float igf_lat, igf_peek;	/* tuning (libvars ig_pflat, ig_pfpeek), read once a think */
/* Position (Billy: the fight planner was "even, slower-paced, worse positioned"):
 * where a dodge leaves the bot on the way to its long-term goal -- the roam's
 * hunt or high-ground point, or the item it was after -- against where it is
 * now (ig_pfroute, per second of travel saved), and the higher ground
 * (ig_pfhigh, at 200+ units above the enemy). igf_tt0: travel time from here. */
static float igf_route, igf_high, igf_near, igf_nearat, igf_range;
static int igf_tt0;

static void IGX_FightSim(igx_t *x, igfopt_t *o, const vec3_t enemyeye, float ready, float skill)
{
	igpm_t pm;
	vec3_t eye, line, side, vh;
	float fm, sm, lat = 0, cost = 0, d, before, after, wallpen = 0;
	int t, half;

	IGX_Start(x, &pm);
	o->ok = 0;
	IGX_Moves(o->dir, x->lastview[YAW], &fm, &sm);
	for (t = 0; t < FIGHT_THINKS; t++) {
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		before = VectorLength(vh);
		pm.blocked = 0;
		for (half = 0; half < 2; half++)
			IG_Pmove(&pm, fm, sm, o->hop && t == 0 ? 400 : 0, x->lastview[PITCH], x->lastview[YAW], 50);
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return;
		if (pm.pad || pm.teleport) return;		/* not mid-fight */
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		after = VectorLength(vh);
		if (pm.blocked && before - after > 150) wallpen += 0.05f;
		/* across the enemy's line: what makes a rail miss */
		VectorSubtract(enemyeye, pm.org, line);
		line[2] = 0;
		d = VectorNormalize(line);
		VectorSet(side, -line[1], line[0], 0);
		lat += fabs(DotProduct(vh, side));
		/* peek: in sight when loaded, out of it when not */
		VectorCopy(pm.org, eye);
		eye[2] += pm.ducked ? -2 : 22;
		if (IGX_Sees(x->bs, eye, enemyeye)) {
			if (ready > (t + 1) * 0.1f) cost += 0.04f * igf_peek;	/* seen while reloading */
		} else {
			if (ready <= (t + 1) * 0.1f) cost += 0.05f;		/* loaded and blind */
		}
		/* range: rails reach anywhere, but too close is a coin toss and too far a duel of luck */
		if (d < 250) cost += 0.02f;
		else if (d > 1400) cost += 0.01f;
		/* closing in: the stock attack move walks to 140 units, and a rail's
		 * aim error is an angle, so the nearer, the surer (ig_pfnear, per
		 * think per 1000 units beyond ig_pfnearat) */
		if (igf_near > 0) {
			if (igf_range > 0) {		/* a range to hold, either side of it */
				float off = fabs(d - igf_range) - 150;

				if (off > 0) cost += igf_near * off / 1000;
			} else if (d > igf_nearat) cost += igf_near * (d - igf_nearat) / 1000;
		}
	}
	if (IGX_EndUnsafe(x, &pm)) return;
	if (igf_route > 0 && x->goalarea && igf_tt0) {
		int tt = pm.area == x->goalarea ? 0 : pm.area ? AAS_AreaTravelTimeToGoalArea(pm.area, pm.org, x->goalarea, x->tfl) : 0;

		if (pm.area == x->goalarea || tt) cost += igf_route * (tt - igf_tt0) * 0.01f;
		else cost += 0.2f;	/* off the route */
	}
	lat /= FIGHT_THINKS;
	cost -= igf_lat * skill * (lat > 450 ? 1.5f : lat / 300);
	/* the higher ground */
	d = pm.org[2] - (enemyeye[2] - 22);
	cost -= igf_high * (d > 200 ? 1 : d < -200 ? -1 : d / 200);
	o->cost = cost + wallpen;
	o->ok = 1;
}

/* A fight that starts mid-flight: the planner's own flight is still the
 * planner's until it lands -- steered, its landing hopped -- and its own hook
 * (a slingshot's) is let go, nothing else would (the fight node does not run
 * the planner). The stock attack move took over 140 of 330 flights from a
 * let-go on q2rdm11, and none of them hopped the landing. */
/* A fight hook let go (ai_ig_sling.c): its flight is the planner's now, so
 * the landing is hopped -- through IGX_FightFlight's simulation, as every
 * planner flight's is. The old code hopped after every fight-hook let-go
 * (IG_Hop), and the planner had dropped it. */
void IG_PlanTakeFlight(int c)
{
	if (c < 0 || c >= MAX_CLIENTS) return;
	igp_state[c].active = 1;
	igp_state[c].landhop = 1;
}

/* The flight on from now, steering along the velocity, jump held coming
 * down when `hop`: lava, slime or a pit before it lands (and hops)? */
static int IGX_FlightUnsafe(bot_state_t *bs, const vec3_t wish, int hop)
{
	igx_t x;
	igpm_t pm;
	float fm, sm;
	int t, half;

	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	IGX_Start(&x, &pm);
	IGX_Moves(wish, bs->viewangles[YAW], &fm, &sm);
	for (t = 0; t < 25; t++) {
		int landed = pm.ground;

		for (half = 0; half < 2; half++)
			IG_Pmove(&pm, fm, sm, hop && pm.vel[2] < 0 ? 400 : 0, 0, bs->viewangles[YAW], 50);
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return 1;
		if (pm.pad || pm.teleport) return 0;
		if (landed && (!hop || pm.jumped)) break;		/* down, and hopped if it was to */
	}
	return IGX_EndUnsafe(&x, &pm);
}

static int IGX_FightFlight(bot_state_t *bs)
{
	int c = bs->client, hs, hop;
	vec3_t eye, wish, hp;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS) return 0;
	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	hs = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), hp);
	if (hs && igp_state[c].hookout) {
		/* our slingshot's hook: let go when that is safe, else hold on (the
		 * fight hook's own rule, IG_ARC_HOLD in ai_ig_sling.c) */
		if (IG_PlanHookUnsafe(bs, 0) && !IG_PlanHookUnsafe(bs, 3)) {
			IG_GrappleTravelNote(c);
			return 0;
		}
		trap_EA_Command(c, LibVarGetString("cmd_grappleoff"));
		igp_state[c].hookout = 0;
		igp_state[c].landhop = 1;
		igp_state[c].active = 1;
	}
	if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE) {
		igp_state[c].active = 0;
		return 0;
	}
	if (hs) return 0;						/* a fight hook's: its own code */
	/* In the air in a fight -- the planner's flight or any other: strafe at
	 * the strafe-jump angle, the side swapped every think so the path runs
	 * straight. Air time in fights was 40% of all a bot's, and it either
	 * pushed along its velocity (no speed in Q2 above 300) or nothing (the
	 * stock attack move), so it gained nothing; Billy gains 4-8 u/s every
	 * 50 ms in the air. */
	{
		vec3_t vh;
		float sp, th, s;

		VectorSet(vh, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		sp = VectorNormalize(vh);
		if (sp < 1) return 0;
		igp_state[c].fightside = -igp_state[c].fightside;
		if (!igp_state[c].fightside) igp_state[c].fightside = 1;
		th = sp > 300 ? IGX_StrafeAngle(sp, IG_StrafeSkill(bs)) : 0;
		s = sin(th) * igp_state[c].fightside;
		VectorSet(wish, vh[0] * cos(th) - vh[1] * s, vh[0] * s + vh[1] * cos(th), 0);
	}
	/* flown first, as everything else: the hop if it is safe, else none, else
	 * the stock attack move and the guard (unchecked, this raised q2rdm11's
	 * lava trips from 0.195 to 0.29 a bot-minute) */
	hop = igp_state[c].landhop;
	if (hop && IGX_FlightUnsafe(bs, wish, 1)) hop = 0;
	if (!hop && IGX_FlightUnsafe(bs, wish, 0)) {
		igp_state[c].active = 0;
		return 0;
	}
	igp_state[c].landhop = hop;
	EA_IGMove(c, wish, 400, hop && bs->cur_ps.velocity[2] < 0, 0);
	igp_state[c].steered = FloatTime();
	return 1;
}

/* IG-40: a pad flight in a fight (Billy, 2026-11-27: "bots are hitting the
 * jump pads but steering away from the intended landing spot"). The fight
 * node runs no travel, so the pad's air control never ran there and the
 * stock attack dodge steered the flight: q2rdm11's twin pads, 16 fight
 * matches, hook-free flights made the ledge 56% (the drill, no fight: 100%),
 * the air steering 114 degrees off the landing on the misses. From the launch
 * to the ground the fight's movement is the pad's own air control
 * (BotAirControl, as BotFinishTravel_JumpPad steers it); the aim and the
 * shots go on, and no fight hook starts (libvar ig_planpadfight, default 1).
 * Out of a fight too (IG_Slingshot): a bot that runs over a pad on its way
 * elsewhere is thrown off its route, and the route's own air control then
 * steered at its goal, not the pad's landing -- the misses that remained. */
static vec3_t igpf_end[MAX_CLIENTS];
static float igpf_until[MAX_CLIENTS];
int BotAirControl(vec3_t origin, vec3_t velocity, vec3_t goal, vec3_t dir, float *speed);

int IG_PlanPadFightActive(int c)
{
	return c >= 0 && c < MAX_CLIENTS && FloatTime() < igpf_until[c];
}

int IG_PlanPadFightFlight(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t end, dir, o, v;
	float speed;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || !LibVarValue("ig_planpadfight", "1")) return 0;
	if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE || bs->cur_ps.pm_type != PM_NORMAL) { igpf_until[c] = 0; return 0; }
	VectorCopy(bs->cur_ps.origin, o);
	VectorCopy(bs->cur_ps.velocity, v);
	if (IG_PadFlight(c, end)) { VectorCopy(end, igpf_end[c]); igpf_until[c] = FloatTime() + 3; }
	else if (FloatTime() >= igpf_until[c]) {
		if (v[2] < 300) return 0;
		if (!IG_PadLaunchEnd(o, end)) {	/* not rising off a pad */
			if (LibVarValue("ig_viewdebug", "0") && v[2] > 600) BotAI_Print(PRT_MESSAGE, "igpadsteer %d no pad under %.0f %.0f %.0f vz %.0f area %d\n", c, o[0], o[1], o[2], v[2], AAS_PointAreaNum(o));
			return 0;
		}
		VectorCopy(end, igpf_end[c]);
		igpf_until[c] = FloatTime() + 3;
		if (LibVarValue("ig_viewdebug", "0")) BotAI_Print(PRT_MESSAGE, "igpadsteer %d node %s end %.0f %.0f %.0f\n", c, igp_node[c], end[0], end[1], end[2]);
	}
	if (!BotAirControl(o, v, igpf_end[c], dir, &speed)) {
		VectorSubtract(igpf_end[c], o, dir); dir[2] = 0;
		VectorNormalize(dir);
		speed = 400;
	}
	EA_Move(c, dir, speed);
	return 1;
}

int IG_PlanFight(bot_state_t *bs)
{
	igx_t x;
	igfopt_t opts[16];
	aas_entityinfo_t enemy, self;
	vec3_t fwd, side, enemyeye, eye;
	float skill, jumper, ready, best = 1e9f, total, r, w[16];
	int c = bs->client, n = 0, i, s, pick, b = -1;
	static const float angles[3] = {0, 45, -45};	/* square, angled in, angled out */

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || BotIsDead(bs) || bs->enemy < 0) return 0;
	if (IGX_FightFlight(bs)) return 1;
	if (!LibVarValue("ig_planfight", "0")) return 0;	/* off: even with stock fights head to head (IG-40) */
	/* testing: only odd client numbers fight with the planner, so the two
	 * fight movements meet in the same matches (ig_pfodd 1) */
	if (LibVarValue("ig_pfodd", "0") && !(c & 1)) return 0;
	if (bs->attackchase_time > FloatTime()) return 0;		/* chasing: the route's */
	skill = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_ATTACK_SKILL, 0, 1);
	jumper = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_JUMPER, 0, 1);
	if (skill < 0.2f) return 0;
	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	VectorCopy(bs->cur_ps.origin, eye);
	eye[2] += bs->cur_ps.viewheight;
	x.hook = IG_OwnHook(eye, VectorLength(bs->cur_ps.velocity), x.hookpos);
	if (x.hook) return 0;									/* a fight hook: its own code */
	if (AAS_PointContents(bs->origin) & (CONTENTS_WATER | CONTENTS_LAVA | CONTENTS_SLIME)) return 0;
	VectorCopy(bs->viewangles, x.lastview);
	if (x.lastview[PITCH] > 180) x.lastview[PITCH] -= 360;

	/* the rail's reload: from this bot's own shots */
	BotEntityInfo(c, &self);
	if (EntityIsShooting(&self) && !igf[c].shooting) igf[c].lastshot = FloatTime();
	igf[c].shooting = EntityIsShooting(&self);
	if (igf[c].lastshot > FloatTime()) igf[c].lastshot = 0;		/* a new map */
	ready = igf[c].lastshot + LibVarValue("ig_planreload", "1.5") - FloatTime();
	if (ready < 0) ready = 0;

	BotEntityInfo(bs->enemy, &enemy);
	if (!enemy.valid) return 0;
	VectorCopy(enemy.origin, enemyeye);
	enemyeye[2] += 22;
	VectorSubtract(enemy.origin, bs->origin, fwd);
	fwd[2] = 0;
	if (VectorNormalize(fwd) < 1) return 0;

	igf_lat = LibVarValue("ig_pflat", "0.12");
	igf_peek = LibVarValue("ig_pfpeek", "1");
	igf_route = LibVarValue("ig_pfroute", "0");
	igf_high = LibVarValue("ig_pfhigh", "0.03");
	igf_near = LibVarValue("ig_pfnear", "0");
	igf_nearat = LibVarValue("ig_pfnearat", "300");
	/* ig_pfstance 1 (Billy: "railgun is about angles and keeping distance
	 * sometimes"): not always closing in. Each fight -- a new enemy, or every
	 * 3-6 s -- picks a range to hold: close 250, mid 550 or long 900, the odds
	 * from the character's aggression (close 0.15 + 0.5 a, long 0.45 - 0.4 a). */
	igf_range = 0;
	if (LibVarValue("ig_pfstance", "0")) {
		if (igf[c].rangeenemy != bs->enemy || FloatTime() > igf[c].rangeuntil || igf[c].rangeuntil > FloatTime() + 10) {
			float a = trap_Characteristic_BFloat(bs->character, CHARACTERISTIC_AGGRESSION, 0, 1), r2 = random();
			float pc = 0.15f + 0.5f * a, pl = 0.45f - 0.4f * a;

			igf[c].range = r2 < pc ? 250 : r2 < 1 - pl ? 550 : 900;
			igf[c].rangeenemy = bs->enemy;
			igf[c].rangeuntil = FloatTime() + 3 + random() * 3;
			if (trap_Cvar_VariableIntegerValue("ig_plandebug") >= 1)
				BotAI_Print(PRT_MESSAGE, "igstance %d range %.0f aggr %.2f\n", c, igf[c].range, a);
		}
		igf_range = igf[c].range;
	}
	igf_tt0 = 0;
	if (igf_route > 0) {
		bot_goal_t goal;

		if (trap_BotGetTopGoal(bs->gs, &goal) && goal.areanum) {
			x.goalarea = goal.areanum;
			VectorCopy(goal.origin, x.goalorg);
			x.tfl = bs->tfl;
			igf_tt0 = bs->areanum == goal.areanum ? 0 : AAS_AreaTravelTimeToGoalArea(bs->areanum, bs->origin, goal.areanum, bs->tfl);
		}
	}
	for (s = -1; s <= 1; s += 2)
		for (i = 0; i < 3; i++) {
			float a = DEG2RAD(90 * s + angles[i] * s);

			VectorSet(side, fwd[0] * cos(a) - fwd[1] * sin(a), fwd[0] * sin(a) + fwd[1] * cos(a), 0);
			VectorCopy(side, opts[n].dir); opts[n].hop = 0; n++;
			if (LibVarValue("ig_pfhop", "1") && (bs->cur_ps.groundEntityNum == ENTITYNUM_NONE || IGX_HSpeed(bs) >= LibVarValue("ig_planhopmin", "250"))) {
				VectorCopy(side, opts[n].dir); opts[n].hop = 1; n++;	/* no hop from a standstill (ig_planhopmin) */
			}
		}
	VectorCopy(fwd, opts[n].dir); opts[n].hop = 0; n++;
	VectorScale(fwd, -1, opts[n].dir); opts[n].hop = 0; n++;

	for (i = 0; i < n; i++) {
		IGX_FightSim(&x, &opts[i], enemyeye, ready, skill);
		if (!opts[i].ok) continue;
		if (i == igf[c].opt && FloatTime() < igf[c].until) opts[i].cost -= 0.15f;	/* keep it a while */
		if (opts[i].cost < best) { best = opts[i].cost; b = i; }
	}
	if (b < 0) return 0;		/* nothing safe: the stock move, which the guard then turns */
	if (b == igf[c].opt && FloatTime() < igf[c].until) {
		pick = b;
	} else {
		/* a fair pick among the near-best; a jumper likes the hops */
		total = 0;
		for (i = 0; i < n; i++) {
			w[i] = opts[i].ok && opts[i].cost <= best + PLAN_MARGIN ? (opts[i].hop ? 0.3f + jumper : 1) : 0;
			total += w[i];
		}
		r = random() * total;
		for (pick = b, i = 0; i < n; i++) {
			if (w[i] <= 0) continue;
			if (r < w[i]) { pick = i; break; }
			r -= w[i];
		}
		igf[c].opt = pick;
		igf[c].until = FloatTime() + 0.3f + random() * 0.5f;
	}
	EA_IGMove(c, opts[pick].dir, 400, opts[pick].hop && bs->cur_ps.groundEntityNum != ENTITYNUM_NONE, 0);
	igp_state[c].steered = FloatTime();
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2)
		BotAI_Print(PRT_MESSAGE, "igfight %d opt %d hop %d cost %.2f best %.2f ready %.2f\n", c, pick, opts[pick].hop, opts[pick].cost, best, ready);
	return 1;
}

/* IG-40 pad top (libvar ig_planpadsling, default 1 since ig-0.5): in a jump pad's flight,
 * the planner may slingshot from the top of the arc along the route past the
 * landing (IGX_PadAnchors); riding the pad on is its "run". Returns as IG_Plan:
 * 0 leaves the flight to the stock pad steering. */
int IG_PlanPadTop(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int r;

	if (!IG_PlanOn() || !LibVarValue("ig_planpadsling", "1") || bs->client < 0 || bs->client >= MAX_CLIENTS) return 0;
	if (!IG_PadFlight(bs->client, igp_padland)) return 0;
	{ bot_goal_t *dg = IG_PadDrillGoal(bs->client); if (dg) goal = dg; }	/* IG-43's pad drill */
	igp_padmode = 1;
	r = IG_Plan(bs, goal, mr);
	igp_padmode = 0;
	return r;
}

/* A pad's flight ends at its first landing. The stock pad travel keeps
 * "finishing" the pad route until a think finds the bot on the ground, and a
 * landing hopped at once never gives it one: each later hop was flown as the
 * pad's (pad mode: ride it out, the stock air steering, no landing hop) and
 * the chain died on Billy's walkway (808 -> 396 u/s). Rising again from below
 * (a jump, not the pad's throw) with no hook out: the pad is done. */
static float igp_lastvz[MAX_CLIENTS];
static int igp_padover[MAX_CLIENTS];

int IG_PlanPadOver(bot_state_t *bs)
{
	int c = bs->client;
	float vz = bs->cur_ps.velocity[2];

	if (c < 0 || c >= MAX_CLIENTS) return 0;
	if (vz > 500 && igp_lastvz[c] < 350) igp_padover[c] = 0;		/* a pad's throw */
	else if (vz > 0 && igp_lastvz[c] <= 0 && !igp_state[c].hookout && LibVarValue("ig_planpadover", "1")) igp_padover[c] = 1;
	if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE && vz < 100) igp_padover[c] = 1;	/* on the launch think the state still stands on the pad */
	igp_lastvz[c] = vz;
	return igp_padover[c];
}

int IG_PlanPadSlingOn(void)
{
	return IG_PlanOn() && LibVarValue("ig_planpadsling", "1") != 0;
}

/* The view in a jump pad's flight (libvar ig_planpadview, default 1, with the
 * planner). The stock pad travel steers the air toward the landing and leaves
 * the view to the roam clamp (be_interface_q2.c), which follows that push as it
 * swings: on q2rdm11's twin pads the view sat 61 degrees off the travel, behind
 * the bot 25% of the flight, turning 226 deg/s (Billy: "they look like they're
 * spinning ... they should be facing the direction they want to land unless
 * they are trying to do a trick or shoot someone"). The bot's input is turned
 * by its view, so a spinning view also spoils the air control. Here: level,
 * toward the landing; within 48 units of it, along the travel. An enemy, a
 * hook's flick or the planner's own steering come first. */
int IG_PlanPadView(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t end, d;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || !LibVarValue("ig_planpadview", "1")) return 0;
	if (igp_state[c].hookout) return 0;
	if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE) {
		/* the last steps onto the pad: turn to the landing now, not in the air */
		vec3_t st;
		if (!IG_PadApproach2(c, st, end)) return 0;
		VectorSubtract(st, bs->origin, d);
		d[2] = 0;
		if (VectorLength(d) > LibVarValue("ig_planpadviewnear", "128")) return 0;
	}
	else if (!IG_PadFlight(c, end)) return 0;
	if (LibVarValue("ig_viewdebug", "0")) BotAI_Print(PRT_MESSAGE, "igpadview %d end %.0f %.0f %.0f\n", c, end[0], end[1], end[2]);
	VectorSubtract(end, bs->origin, d);
	d[2] = 0;
	if (VectorLength(d) < 48) {
		VectorSet(d, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		if (VectorLength(d) < 50) return 0;
	}
	bs->ideal_viewangles[YAW] = AngleMod(atan2(d[1], d[0]) * 180 / M_PI);
	bs->ideal_viewangles[PITCH] = 0;
	return 1;
}

/* Air control in a pad's flight (libvar ig_planpadair, default 0 since Billy's A/B, with the
 * planner). The stock pad steering (BotAirControl) pushes at full strength
 * at the landing it predicts without the push itself, so it overcorrects: on
 * q2rdm11's twin pads bots drifted past the landing into the side wall and
 * fell (Billy: "they're not bouncing to the optimal spot"). Here, each think:
 * no push, or one of 16 directions held for the rest of the flight, each flown
 * by the simulator; the one that lands nearest the pad's landing (a landing
 * below it priced x3 per unit; lava out) is the input. */
int IG_PlanPadAir(bot_state_t *bs)
{
	int c = bs->client, k, t, half, best = -2;
	vec3_t end, wish, bestwish = {0, 0, 0}, d;
	float bestd = 1e9f, fm, sm, yaw = bs->viewangles[YAW];
	igpm_t pm;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS) return 0;
	if (bs->cur_ps.groundEntityNum != ENTITYNUM_NONE || igp_state[c].hookout) return 0;
	if (!IG_PadFlight(c, end)) return 0;
	if (LibVarValue("ig_viewdebug", "0")) BotAI_Print(PRT_MESSAGE, "igpadend %d end %.0f %.0f %.0f\n", c, end[0], end[1], end[2]);
	if (!LibVarValue("ig_planpadair", "0")) return 0;
	for (k = -1; k < 16; k++) {
		float dist;

		if (k < 0) VectorClear(wish);
		else VectorSet(wish, cos(k * M_PI / 8), sin(k * M_PI / 8), 0);
		IG_PmoveInit(&pm, bs->cur_ps.origin, bs->cur_ps.velocity, 0, bs->entitynum, 0);
		IGX_Moves(wish, yaw, &fm, &sm);
		for (t = 0; t < 60 && !pm.ground; t++)
			for (half = 0; half < 2 && !pm.ground; half++)
				IG_Pmove(&pm, fm, sm, 0, 0, yaw, 25);
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
		VectorSubtract(end, pm.org, d);
		dist = sqrt(d[0] * d[0] + d[1] * d[1]);
		if (end[2] - pm.org[2] > 32) dist += 3 * (end[2] - pm.org[2] - 32);
		if (dist < bestd - 1) { bestd = dist; best = k; VectorCopy(wish, bestwish); }
	}
	if (best < -1) return 0;
	EA_Move(c, bestwish, best < 0 ? 0 : 400);
	if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 4)
		BotAI_Print(PRT_MESSAGE, "igpadair %d dir %d miss %.0f\n", c, best, bestd);
	return 1;
}

/* The view on a flight the planner steers (libvar ig_planflightview, default
 * 1). With no enemy the stock look-ahead (or a random roam spot), clamped near
 * the push direction (be_interface_q2.c), chased a target that jumped around
 * while the push sat 70-90 degrees off the travel at speed: the view turned at
 * its 360 deg/s limit nonstop and the model spun, strafing sideways or
 * backwards (Billy: "the model converts into strafe jumping backwards"). A
 * player holds forward and a side key, so his view is 45 degrees from the push,
 * toward where he is going: that, and level. */
int IG_PlanFlightView(bot_state_t *bs)
{
	int c = bs->client;
	vec3_t vh;
	float sp, vyaw, wyaw, d;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || !LibVarValue("ig_planflightview", "2")) return 0;
	if (FloatTime() - igp_state[c].steered > 0.001f || !igp_state[c].setmove) return 0;
	VectorSet(vh, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
	sp = VectorNormalize(vh);
	if (sp < 250) return 0;
	vyaw = atan2(vh[1], vh[0]) * 180 / M_PI;
	wyaw = atan2(igp_state[c].wish[1], igp_state[c].wish[0]) * 180 / M_PI;
	d = AngleMod(wyaw - vyaw + 180) - 180;	/* the push's side of the travel */
	/* Billy's route: his view 5 degrees (median) off his travel, the strafe in
	 * the keys; mode 2 (default): the travel, leaning a little toward the push;
	 * mode 1: 45 degrees from the push (28 degrees off the travel) */
	if (LibVarValue("ig_planflightview", "2") >= 2) bs->ideal_viewangles[YAW] = AngleMod(vyaw + d * 0.1f);
	else bs->ideal_viewangles[YAW] = AngleMod(fabs(d) < 45 ? vyaw : wyaw - (d > 0 ? 45 : -45));
	bs->ideal_viewangles[PITCH] = 0;
	return 1;
}
