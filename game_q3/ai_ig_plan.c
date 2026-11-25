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

#define PLAN_THINKS		12		/* 1.2 s ahead */
#define PLAN_MARGIN		0.06f	/* options this close to the best are all fair picks (s) */
#define PLAN_KEEP		0.04f	/* the option under way gets this much benefit of the doubt (s) */
#define PLAN_COMMIT		0.25f	/* a flick is a promise: its fire, next think, unless that turned bad (s) */
#define PLAN_SPEEDBONUS	0.35f	/* s saved per 300 u/s carried along the route beyond running speed */
/* A hook has a price: a seasoned player does not hook for a 2% gain. In
 * seconds, scaled by the bot's hook affinity (a grapple lover pays less):
 * libvar ig_planhookcost (s at the middle affinity, 0.55). */
#define PLAN_COAST		15		/* up to 1.5 s more, running the route, until the extra speed is spent */
#define PLAN_MAXANCH	16
#define PLAN_MAXOPT		32

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

/* The route's way from org. *special: the route's next step there is one
 * the simulation does not model -- a gap jump, a barrier jump, a ladder, a
 * lift, a climb, a pad, a teleporter: the stock movement takes those, and a
 * simulated run would walk off the gap's edge (brutalo, q2rdm4: upper level
 * time fell 4-7 points while it did). */
static int IGX_RouteDirS(igx_t *x, const vec3_t org, int area, vec3_t dir, int *special)
{
	int tt, rn, tv;
	aas_reachability_t r;
	vec3_t t, o;

	if (special) *special = 0;
	if (!area) return 0;
	VectorCopy(org, o);
	if (area == x->goalarea) {
		VectorCopy(x->goalorg, t);
	} else {
		if (!AAS_AreaRouteToGoalArea(area, o, x->goalarea, x->tfl, &tt, &rn) || !rn) return 0;
		AAS_ReachabilityFromNum(rn, &r);
		tv = r.traveltype & TRAVELTYPE_MASK;
		if (special && tv != TRAVEL_WALK && tv != TRAVEL_WALKOFFLEDGE) *special = 1;
		VectorCopy(r.start, t);
		/* at the reachability's start already: head across it */
		if ((r.start[0] - o[0]) * (r.start[0] - o[0]) + (r.start[1] - o[1]) * (r.start[1] - o[1]) < 40 * 40)
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
		in->jump = (t == 0 && o->jump0) || o->hop || (o->landhop && (x->released || in->letgo) && pm->jumps == x->reljumps && (in->letgo || pm->hook < 2));
	} else {
		if (pm->hook >= 2) VectorCopy(routedir, in->wish);
		else IGX_AirWish(pm->vel, routedir, in->wish, x->strafeskill);
		/* pressed while coming down, the jump goes off on the landing frame:
		 * Billy's slingshot (demo, 2026-11-25) lets go in the air and has jump
		 * down before he lands -- off the floor again within 8 ms, 860-940 u/s
		 * kept; a landing without it gives friction the first step */
		in->jump = (o->hop || (o->landhop && (x->released || in->letgo) && pm->jumps == x->reljumps)) && pm->vel[2] < 0 && (pm->hook < 2 || in->letgo);
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
		}
		VectorCopy(in.view, x->lastview);	/* the game fires along this next think */
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return;	/* out */
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		after = VectorLength(vh);
		if (pm.blocked && before - after > 200) wallpen += 0.15f * (before - after) / 300;
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
			if (pm.blocked && before - after > 200) wallpen += 0.15f * (before - after) / 300;
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
enum { IGA_SLING, IGA_UP, IGA_SIDE, IGA_PADTOP };
static const char *iga_names[] = { "sling", "up", "side", "padtop" };

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
	if (cls == IGA_SLING && tr.plane.normal[2] <= 0.7f && VectorLength(d) < 600) return;
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
static void IGX_PadAnchors(igx_t *x, const vec3_t routedir)
{
	static const float yaws[3] = {-16, 0, 16}, pitches[5] = {6, 2, -3, -8, -12};
	float baseyaw = atan2(routedir[1], routedir[0]) * 180 / M_PI;
	int i, j;

	x->numanchors = 0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 5; j++)
			IGX_AnchorTry(x, pitches[j], baseyaw + yaws[i], 300, IGA_PADTOP);
}

static float IGX_ClassMul(int cls)
{
	if (cls == IGA_PADTOP) return LibVarValue("ig_planpadmul", "0.3");
	return cls == IGA_SLING ? LibVarValue("ig_planslingmul", "0.3") :
		   cls == IGA_UP ? LibVarValue("ig_planupmul", "1") : LibVarValue("ig_plansidemul", "2");
}

/* ------------------------------------------------------------- the plan */

int IG_PlanOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_plan") && trap_Cvar_VariableIntegerValue("ig_hookdetect");
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
static vec3_t	igp_padland;

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
	x.tfl = bs->tfl;
	x.jumpland = trap_Cvar_VariableIntegerValue("ig_jumpland");
	x.canfire = IG_CanFireHook(c);
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

	/* what the planner does not plan: the stock movement's */
	if (!igp_padmode && !ours && ((mr->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK ||
				  (mr->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE))))
		return 0;
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
		if (!la || !IGX_RouteDir(&x, up, la, routedir)) {
			/* the landing's own area has no step (often the goal's): toward the goal */
			VectorSubtract(goal->origin, igp_padland, routedir);
			routedir[2] = 0;
			if (VectorNormalize(routedir) < 64) {
				if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 3) BotAI_Print(PRT_MESSAGE, "igpad %d noroute area %d\n", c, la);
				return 0;
			}
		}
	} else if (!IGX_RouteDir(&x, pm.org, pm.area, routedir)) {
		if (!ours) return 0;
		VectorSet(routedir, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		if (VectorNormalize(routedir) < 1) return 0;
	}

	/* the options from here */
#define OPT(K, H, J0, A, F, R, L) do { if (n < PLAN_MAXOPT) { igopt_t *o_ = &opts[n++]; o_->kind = K; o_->hop = H; o_->hookmul = 1; o_->landhop = 0; \
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
				IGX_PadAnchors(&x, routedir);
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
		EA_IGMove(c, in.wish, 400, in.jump, 0);
		igp_state[c].steered = FloatTime();
		igp_state[c].setmove = 1;
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
			IG_GrappleTravelNote(c);
			igp_state[c].hookout = 1;
			if (igp_padmode) igp_state[c].padfired = 1;
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

/* IG-40 pad top (libvar ig_planpadsling, default 0): in a jump pad's flight,
 * the planner may slingshot from the top of the arc along the route past the
 * landing (IGX_PadAnchors); riding the pad on is its "run". Returns as IG_Plan:
 * 0 leaves the flight to the stock pad steering. */
int IG_PlanPadTop(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	int r;

	if (!IG_PlanOn() || !LibVarValue("ig_planpadsling", "0") || bs->client < 0 || bs->client >= MAX_CLIENTS) return 0;
	if (!IG_PadFlight(bs->client, igp_padland)) return 0;
	igp_padmode = 1;
	r = IG_Plan(bs, goal, mr);
	igp_padmode = 0;
	return r;
}

int IG_PlanPadSlingOn(void)
{
	return IG_PlanOn() && LibVarValue("ig_planpadsling", "0") != 0;
}
