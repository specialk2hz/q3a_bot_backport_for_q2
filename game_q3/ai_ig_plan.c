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
#define PLAN_SPEEDBONUS	0.35f	/* s saved per 300 u/s carried along the route beyond running speed */
/* A hook has a price: a seasoned player does not hook for a 2% gain. In
 * seconds, scaled by the bot's hook affinity (a grapple lover pays less):
 * libvar ig_planhookcost (s at the middle affinity, 0.55). */
#define PLAN_MAXANCH	12
#define PLAN_MAXOPT		32

enum { PO_RUN, PO_HOP, PO_SLING, PO_FIRE, PO_HOLD, PO_LETGO, PO_LETGOHOP };
static const char *po_names[] = { "run", "hop", "sling", "fire", "hold", "letgo", "letgohop" };

typedef struct {
	int		kind;
	int		hop;		/* strafe-hop chain for the whole run */
	int		jump0;		/* jump on the first think */
	int		anchor;		/* flick onto anchors[anchor] on the first think; -1 none */
	int		fire;		/* think to fire at; -1 none */
	int		relafter;	/* let go this many thinks after the hook bites; 0 keep it */
	int		letgo;		/* let go on the first think */
	float	cost;
	int		ok;
} igopt_t;

typedef struct {
	bot_state_t	*bs;
	int			goalarea, tfl, jumpland, canfire;
	vec3_t		goalorg;
	vec3_t		anchors[PLAN_MAXANCH];
	vec3_t		anchorview[PLAN_MAXANCH];	/* pitch, yaw from the eye now */
	int			numanchors;
	vec3_t		lastview;					/* the view the game holds now (last usercmd) */
	int			hook;						/* own hook now: 0, 1 flying, 2 still */
	float		hookcost;					/* the price of firing a hook, s */
	vec3_t		hookpos, hookdir;
} igx_t;

static struct {
	int		active;			/* something of ours under way: a hop, a sling, our hook */
	int		kind, anchorset;
	vec3_t	anchor, flickview;
	float	flicktime;
	int		hookout;		/* we fired the hook that is out */
	vec3_t	firedir;		/* ...along this: the flight a flying hook keeps */
	int		chain;
	float	lastlog;
	float	steered;	/* the think the planner set the input itself */
	float	viewset;	/* the think the planner set the view (a flick, or holding one) */
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

static int IGX_RouteDir(igx_t *x, const vec3_t org, int area, vec3_t dir)
{
	int tt, rn;
	aas_reachability_t r;
	vec3_t t, o;

	if (!area) return 0;
	VectorCopy(org, o);
	if (area == x->goalarea) {
		VectorCopy(x->goalorg, t);
	} else {
		if (!AAS_AreaRouteToGoalArea(area, o, x->goalarea, x->tfl, &tt, &rn) || !rn) return 0;
		AAS_ReachabilityFromNum(rn, &r);
		VectorCopy(r.start, t);
		/* at the reachability's start already: head across it */
		if ((r.start[0] - o[0]) * (r.start[0] - o[0]) + (r.start[1] - o[1]) * (r.start[1] - o[1]) < 40 * 40)
			VectorCopy(r.end, t);
	}
	VectorSubtract(t, o, dir);
	dir[2] = 0;
	return VectorNormalize(dir) > 1;
}

/* In the air: the wish that keeps the most speed while turning toward the
 * route (Q2's air accel 1 adds speed only within acos(285/speed) of the
 * velocity -- the strafe-jump angle). */
static void IGX_AirWish(const vec3_t vel, const vec3_t route, vec3_t wish)
{
	vec3_t vh;
	float sp, theta, a, side, c, s;

	VectorSet(vh, vel[0], vel[1], 0);
	sp = VectorNormalize(vh);
	if (sp < 200) {
		VectorCopy(route, wish);
		return;
	}
	theta = acos(285.0f / sp > 1 ? 1 : 285.0f / sp);
	a = acos(DotProduct(vh, route) > 1 ? 1 : DotProduct(vh, route) < -1 ? -1 : DotProduct(vh, route));
	if (a <= theta) {
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

	if (pm->ground) {
		VectorCopy(routedir, in->wish);
		in->jump = (t == 0 && o->jump0) || o->hop;
	} else {
		if (pm->hook >= 2) VectorCopy(routedir, in->wish);
		else IGX_AirWish(pm->vel, routedir, in->wish);
		/* pressed while coming down, the jump goes off on the landing frame */
		in->jump = o->hop && pm->vel[2] < 0 && pm->hook < 2;
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

static void IGX_Simulate(igx_t *x, igopt_t *o)
{
	igpm_t pm;
	igin_t in;
	vec3_t route, lastroute, end, vh;
	int t, latch = -1, airjump = -1, half, area, tt;
	float fm, sm, up, wallpen = 0, along, before, after;

	IGX_Start(x, &pm);
	o->ok = 0;
	if (!IGX_RouteDir(x, pm.org, pm.area, lastroute)) {
		AngleVectors(x->bs->viewangles, lastroute, NULL, NULL);
		lastroute[2] = 0;
		VectorNormalize(lastroute);
	}
	for (t = 0; t < PLAN_THINKS; t++) {
		if (pm.hook >= 2 && latch < 0) latch = t;
		if (IGX_RouteDir(x, pm.org, pm.area, route)) VectorCopy(route, lastroute);
		else VectorCopy(lastroute, route);
		IGX_Input(x, o, t, &pm, route, latch, &in);
		if (in.letgo) IG_PmoveHookRelease(&pm);
		if (in.fire) {
			IG_PmoveHookFire(&pm, x->lastview[PITCH], x->lastview[YAW]);
			wallpen += x->hookcost;
		}
		IGX_Moves(in.wish, in.view[YAW], &fm, &sm);
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		before = VectorLength(vh);
		pm.blocked = 0;
		for (half = 0; half < 2; half++) {
			up = in.jump ? 400 : 0;
			/* the game half of strafe chains (ig_botjumpland): a jump pressed
			 * in the air is held two more frames */
			if (x->jumpland) {
				if (pm.ground || pm.hook) airjump = -1;
				else if (up > 0) airjump = t;
				else if (airjump >= 0 && t - airjump <= 2) up = 400;
			}
			IG_Pmove(&pm, fm, sm, up, in.view[PITCH], in.view[YAW], 50);
		}
		VectorCopy(in.view, x->lastview);	/* the game fires along this next think */
		if (pm.everhit & (CONTENTS_LAVA | CONTENTS_SLIME)) return;	/* out */
		VectorSet(vh, pm.vel[0], pm.vel[1], 0);
		after = VectorLength(vh);
		if (pm.blocked && before - after > 200) wallpen += 0.15f * (before - after) / 300;
		if (pm.pad || pm.teleport) break;	/* the navigation file knows where those go */
	}
	if (pm.hook == 1 && pm.hooklost) wallpen += 0.05f;	/* a wasted hook */

	/* where it ends: the route's time from there */
	area = pm.area;
	VectorCopy(pm.org, end);
	if (!pm.ground && !pm.pad && !pm.teleport) {	/* still in the air: where it comes down */
		bsp_trace_t tr;
		vec3_t down, mins = {-16, -16, -24}, maxs = {16, 16, 32};

		VectorCopy(end, down);
		down[2] -= 1000;
		BotAI_Trace(&tr, end, mins, maxs, down, x->bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP | CONTENTS_LAVA | CONTENTS_SLIME);
		if (tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME))) return;	/* a pit, or lava below */
		VectorCopy(tr.endpos, end);
		area = AAS_PointAreaNum(end);
	}
	if (!area) return;
	if (area == x->goalarea) tt = 0;
	else if (!(tt = AAS_AreaTravelTimeToGoalArea(area, end, x->goalarea, x->tfl))) return;
	VectorSet(vh, pm.vel[0], pm.vel[1], 0);
	if (!IGX_RouteDir(x, end, area, route)) VectorCopy(lastroute, route);
	along = DotProduct(vh, route);
	o->cost = pm.time + tt * 0.01f - PLAN_SPEEDBONUS * (along > 300 ? (along > 750 ? 1.5f : (along - 300) / 300) : 0) + wallpen;
	o->ok = 1;
}

/* -------------------------------------------------------------- anchors */

static void IGX_Anchors(igx_t *x, const vec3_t routedir)
{
	/* + is down; the -25 row is the stairwell and ledge hook (Billy's T6) */
	static const float yaws[3] = {-25, 0, 25}, pitches[4] = {20, 8, -8, -25};
	bot_state_t *bs = x->bs;
	float baseyaw = atan2(routedir[1], routedir[0]) * 180 / M_PI, maxd = 900;
	int i, j, sky = trap_Cvar_VariableIntegerValue("ig_hooksky");
	vec3_t ang, dir, end, d;
	bsp_trace_t tr;

	x->numanchors = 0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 4; j++) {
			VectorSet(ang, pitches[j], baseyaw + yaws[i], 0);
			AngleVectors(ang, dir, NULL, NULL);
			VectorMA(bs->eye, maxd, dir, end);
			BotAI_Trace(&tr, bs->eye, NULL, NULL, end, bs->entitynum, CONTENTS_SOLID | CONTENTS_LAVA | CONTENTS_SLIME);
			if (tr.fraction >= 1 || tr.startsolid) continue;
			if (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			if ((tr.surface.flags & SURF_SKY) && !sky) continue;
			VectorSubtract(tr.endpos, bs->eye, d);
			if (VectorLength(d) < 200) continue;
			VectorCopy(tr.endpos, x->anchors[x->numanchors]);
			vectoangles(d, x->anchorview[x->numanchors]);
			if (x->anchorview[x->numanchors][PITCH] > 180) x->anchorview[x->numanchors][PITCH] -= 360;
			x->numanchors++;
		}
}

/* ------------------------------------------------------------- the plan */

int IG_PlanOn(void)
{
	return trap_Cvar_VariableIntegerValue("ig_plan") && trap_Cvar_VariableIntegerValue("ig_hookdetect");
}

static void IGX_Flick(bot_state_t *bs, const vec3_t view)
{
	int i;

	if (bs->client >= 0 && bs->client < MAX_CLIENTS) igp_state[bs->client].viewset = FloatTime();
	VectorCopy(view, bs->ideal_viewangles);
	bs->flags |= BFL_IDEALVIEWSET;
	for (i = 0; i < 2; i++) {
		bs->viewangles[i] = AngleMod(view[i]);
		bs->viewanglespeed[i] = 0;
	}
}

int IG_Plan(bot_state_t *bs, bot_goal_t *goal, bot_moveresult_t *mr)
{
	igx_t x;
	igopt_t opts[PLAN_MAXOPT];
	int n = 0, i, best = -1, pick, c = bs->client, ground, ours, k;
	float bestcost = 1e9f, total, r, w[PLAN_MAXOPT];
	vec3_t eye, routedir, hookpos;
	igpm_t pm;
	igin_t in;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || BotIsDead(bs) || !goal || !goal->areanum) return 0;
	igp_lastplan[c] = FloatTime();
	Com_Memset(&x, 0, sizeof(x));
	x.bs = bs;
	x.goalarea = goal->areanum;
	VectorCopy(goal->origin, x.goalorg);
	x.tfl = bs->tfl;
	x.jumpland = trap_Cvar_VariableIntegerValue("ig_jumpland");
	x.canfire = IG_CanFireHook(c);
	x.hookcost = LibVarValue("ig_planhookcost", "0.3") * (1.6f - IG_PlanAffinity(bs)) / (1.6f - 0.55f);
	VectorCopy(bs->viewangles, x.lastview);
	if (x.lastview[PITCH] > 180) x.lastview[PITCH] -= 360;
	if (igp_state[c].flicktime > 0 && FloatTime() - igp_state[c].flicktime < 0.15f)
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

	/* what the planner does not plan: the stock movement's */
	if (!ours && ((mr->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK ||
				  (mr->flags & (MOVERESULT_MOVEMENTVIEWSET | MOVERESULT_WAITING | MOVERESULT_ONTOPOFOBSTACLE))))
		return 0;
	if (x.hook && !igp_state[c].hookout) return 0;		/* a climb route's hook: not ours */
	if (AAS_PointContents(bs->origin) & (CONTENTS_WATER | CONTENTS_LAVA | CONTENTS_SLIME)) return 0;

	IGX_Start(&x, &pm);
	if (!IGX_RouteDir(&x, pm.org, pm.area, routedir)) {
		if (!ours) return 0;
		VectorSet(routedir, bs->cur_ps.velocity[0], bs->cur_ps.velocity[1], 0);
		if (VectorNormalize(routedir) < 1) return 0;
	}

	/* the options from here */
#define OPT(K, H, J0, A, F, R, L) do { if (n < PLAN_MAXOPT) { igopt_t *o_ = &opts[n++]; o_->kind = K; o_->hop = H; \
	o_->jump0 = J0; o_->anchor = A; o_->fire = F; o_->relafter = R; o_->letgo = L; } } while (0)
	if (x.hook) {
		OPT(PO_LETGO, 0, 0, -1, -1, 0, 1);
		OPT(PO_LETGOHOP, 1, 0, -1, -1, 0, 1);
		for (k = 1; k <= 4; k++) OPT(PO_HOLD, 0, 0, -1, -1, k * k == 16 ? 8 : k, 0);	/* 1, 2, 3, 8 thinks */
	} else {
		OPT(PO_RUN, 0, 0, -1, -1, 0, 0);
		OPT(PO_HOP, 1, ground, -1, -1, 0, 0);
		/* flicked onto an anchor last think: it can go now */
		if (igp_state[c].anchorset && FloatTime() - igp_state[c].flicktime < 0.15f && x.canfire) {
			OPT(PO_FIRE, 0, 0, -1, 0, 3, 0);
		}
		if (x.canfire) {
			IGX_Anchors(&x, routedir);
			for (i = 0; i < x.numanchors; i++)
				OPT(PO_SLING, 0, ground, i, 1, 3, 0);
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
		if (opts[i].kind == igp_state[c].kind || (opts[i].kind == PO_FIRE && igp_state[c].kind == PO_SLING))
			opts[i].cost -= PLAN_KEEP;
		if (opts[i].cost < bestcost) {
			bestcost = opts[i].cost;
			best = i;
		}
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
		w[i] = opts[i].kind == PO_SLING || opts[i].kind == PO_FIRE ? IG_PlanAffinity(bs) * 2 : 1;
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
				return 1;
			}
			igp_state[c].active = 0;
			return 0;	/* the stock route move */
		}
		EA_IGMove(c, in.wish, 400, in.jump, 0);
		igp_state[c].steered = FloatTime();
		if (o->anchor >= 0) {
			IGX_Flick(bs, x.anchorview[o->anchor]);
			VectorCopy(x.anchorview[o->anchor], igp_state[c].flickview);
			VectorCopy(x.anchors[o->anchor], igp_state[c].anchor);
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
			igp_state[c].anchorset = 0;
			AngleVectors(x.lastview, igp_state[c].firedir, NULL, NULL);
			VectorSubtract(igp_state[c].anchor, eye, d);
			BotAI_Print(PRT_MESSAGE, "ig: slingshot %d length %.0f chain %d air %d\n", c, VectorLength(d),
						igp_state[c].chain++, !ground);
			if (trap_Cvar_VariableIntegerValue("ig_plandebug") == 2)
				BotAI_Print(PRT_MESSAGE, "igfire %d view %.1f %.1f anchor %.0f %.0f %.0f eye %.0f %.0f %.0f\n", c,
							x.lastview[PITCH], x.lastview[YAW], igp_state[c].anchor[0], igp_state[c].anchor[1],
							igp_state[c].anchor[2], eye[0], eye[1], eye[2]);
		}
		if (x.hook) IG_GrappleTravelNote(c);	/* ours: the watchdog keeps off */
		if (ground && o->kind != PO_HOLD) igp_state[c].chain = 0;
		igp_state[c].kind = o->kind;
		igp_state[c].active = 1;
		if (trap_Cvar_VariableIntegerValue("ig_plandebug") && (o->kind != PO_HOLD || FloatTime() - igp_state[c].lastlog > 0.5f)) {
			igp_state[c].lastlog = FloatTime();
			BotAI_Print(PRT_MESSAGE, "ig: plan %d %s cost %.2f best %.2f options %d\n", c, po_names[o->kind], o->cost, bestcost, n);
		}
	}
	return 1;
}

/* ---------------------------------------------------------------- guard */

/* Does holding this input for GUARD_THINKS end in lava, slime or a pit?
 * The body as it is now, the bot's own hook included. */
#define GUARD_THINKS	5
static int IGX_Unsafe(igx_t *x, const vec3_t dir, float speed, int jump, int crouch, float yaw, float pitch)
{
	igpm_t pm;
	vec3_t wish, end, down, mins = {-16, -16, -24}, maxs = {16, 16, 32};
	float fm, sm;
	int t, half;
	bsp_trace_t tr;

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
	if (pm.ground) return 0;
	VectorCopy(pm.org, end);
	VectorCopy(end, down);
	down[2] -= 1000;
	BotAI_Trace(&tr, end, mins, maxs, down, x->bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP | CONTENTS_LAVA | CONTENTS_SLIME);
	return tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME));
}

/* Every think, after the AI has made its input (be_interface_q2.c): an input
 * the planner did not make itself -- a fight's dodges, the stock route move --
 * that would take the bot into lava or a pit within half a second is changed
 * for the nearest safe one: the same without the jump, then turned 45, 90 and
 * 135 degrees either way, then still. Combat strafing walked bots off
 * q2rdm11's walkways into the lava; stopping or reversing outright, the first
 * version of this, cost brutalo's platform fights 40 u/s. */
void IG_PlanGuard(bot_state_t *bs, bot_input_t *bi)
{
	igx_t x;
	vec3_t eye, back, zero = {0, 0, 0};
	int c = bs->client, jump, crouch;
	float pitch;

	if (!IG_PlanOn() || c < 0 || c >= MAX_CLIENTS || BotIsDead(bs)) return;
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
	vec3_t down, mins = {-16, -16, -24}, maxs = {16, 16, 32};
	bsp_trace_t tr;

	if (pm->ground || pm->pad || pm->teleport) return 0;
	VectorCopy(pm->org, down);
	down[2] -= 1000;
	BotAI_Trace(&tr, pm->org, mins, maxs, down, x->bs->entitynum, CONTENTS_SOLID | CONTENTS_PLAYERCLIP | CONTENTS_LAVA | CONTENTS_SLIME);
	return tr.fraction >= 1 || (tr.contents & (CONTENTS_LAVA | CONTENTS_SLIME));
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

