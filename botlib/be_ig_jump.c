/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_jump.c -- IG-46: running jumps across gaps bspc left out
 * (libvar ig_gapjumps, default 0; read at map load).
 *
 * q2rdm11's upper walkway has a 192-unit gap (x 1056-1247 at y ~1100, the
 * floor 390 below). Billy hops it at 700 u/s on his pad line; bspc made no
 * jump across it, so from the pad's landing east of it the routing goes back
 * through the teleporter (8.3 s) and the bots never run his line.
 *
 * Here, at load, before the routing: from the middle of each floor area, in
 * 16 directions, walk to where the floor ends (a wall first: nothing); look
 * for floor at the same height (48 below to 18 above) 40..ig_gapjumpmax (260)
 * units on; if the gap has a floor or a fall under it (not lava, slime or the
 * void) and no route already joins the two areas, fly the jump -- run speed
 * ig_gapjumpspeed (300) at the edge, the game's jump velocity, its gravity,
 * the player's box -- and if it lands on that floor, add a TRAVEL_JUMP route
 * (time: the flight + ig_gapjumpprice 0.3 s). Same-cluster only (the routing
 * ignores a route out of its cluster). The area's block of routes moves to the
 * end of the table, as IG-43's pad routes do, so no route number changes. */

#include "../game_q3/q_shared.h"
#include "l_memory.h"
#include "l_libvar.h"
#include "aasfile.h"
#include "../game_q3/botlib.h"
#include "../game_q3/be_aas.h"
#include "be_aas_funcs.h"
#include "be_interface.h"
#include "be_aas_def.h"
#include "be_ig_hook.h"

#define IG_GJ_DIRS 16
#define IG_GJ_MAX  4096

typedef struct { int from; aas_reachability_t r; float gap, need; } igjump_t;

/* IG-47: a bot may take momentum jumps (TRAVEL_STRAFEJUMP) from ig_fastjumpon
 * (500) u/s and keeps the flag until it slows under ig_fastjumpoff (380), so
 * its route does not flicker between the jump and the long way round. */
static int igfj_on[MAX_CLIENTS];
static float igfj_onspd = 500, igfj_offspd = 380;
static int igfj_enabled;

/* Variety (Billy, 2026-11-28: "try to make it 50/50 lip grapple and strafe
 * jumping stairs"): every ig_fastjumproll (8) s a bot rolls whether it takes
 * momentum jumps for a while, at ig_fastjumpchance (0.5) */
static float igfj_rolled[MAX_CLIENTS];
static int igfj_take[MAX_CLIENTS];
int IG_FastJumpAllowed(int client)
{
	float now = AAS_Time();
	if (!igfj_enabled || client < 0 || client >= MAX_CLIENTS) return 0;
	if (now < igfj_rolled[client] || now - igfj_rolled[client] > LibVarValue("ig_fastjumproll", "8")) {
		igfj_rolled[client] = now;
		igfj_take[client] = random() < LibVarValue("ig_fastjumpchance", "0.5");
	}
	return igfj_take[client];
}

int IG_FastJumpTfl(int client, const vec3_t vel)
{
	float s;
	if (!igfj_enabled || client < 0 || client >= MAX_CLIENTS) return 0;
	if (!IG_FastJumpAllowed(client)) return 0;
	s = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
	if (s >= igfj_onspd) igfj_on[client] = 1;
	else if (s < igfj_offspd) igfj_on[client] = 0;
	return igfj_on[client] ? TFL_STRAFEJUMP : 0;
}

static int IGJ_Ground(vec3_t p, float down, vec3_t floor)
{
	vec3_t a, b;
	aas_trace_t tr;

	VectorCopy(p, a); a[2] += 4;
	VectorCopy(p, b); b[2] -= down;
	tr = AAS_TraceClientBBox(a, b, PRESENCE_NORMAL, -1);
	if (tr.startsolid || tr.fraction >= 1) return 0;
	VectorCopy(tr.endpos, floor);
	return 1;
}

static int IGJ_HasRoute(int from, int to)
{
	aas_areasettings_t *s = &aasworld.areasettings[from];
	int i;

	for (i = 0; i < s->numreachableareas; i++)
		if (aasworld.reachability[s->firstreachablearea + i].areanum == to) return 1;
	return 0;
}

/* fly the jump from e along d; 1 and the landing if it comes down in area want */
static int IGJ_Fly(vec3_t e, vec3_t d, float speed, int want, vec3_t land, float *time)
{
	vec3_t pos, next, vel;
	float t, dt = 0.025f, g = aassettings.phys_gravity > 0 ? aassettings.phys_gravity : 800;
	float jv = aassettings.phys_jumpvel > 0 ? aassettings.phys_jumpvel : 270;
	aas_trace_t tr;

	VectorCopy(e, pos);
	VectorScale(d, speed, vel);
	vel[2] = jv;
	for (t = 0; t < 1.5f; t += dt) {
		VectorMA(pos, dt, vel, next);
		vel[2] -= g * dt;
		tr = AAS_TraceClientBBox(pos, next, PRESENCE_NORMAL, -1);
		if (tr.startsolid) return 0;
		if (tr.fraction < 1) {
			if (vel[2] >= 0) return 0;				/* hit a wall or a ceiling going up */
			if (!IGJ_Ground(tr.endpos, 8, land)) return 0;	/* a wall on the way down */
			*time = t + dt * tr.fraction;
			return AAS_PointAreaNum(land) == want || AAS_PointAreaNum(next) == want;
		}
		VectorCopy(next, pos);
	}
	return 0;
}

static void IGJ_Rej(int debug, const char *why, vec3_t e, int k)
{
	float wx = atof(LibVarString("ig_gapjumpwatchx", "0")), wy = atof(LibVarString("ig_gapjumpwatchy", "0"));
	if (debug >= 2 && fabs(e[0] - wx) < 96 && fabs(e[1] - wy) < 96)
		botimport.Print(PRT_MESSAGE, "ig: gapjump reject %s at %.0f %.0f %.0f dir %d\n", why, e[0], e[1], e[2], k);
}

void IG_GapJumpsAtLoad(void)
{
	static igjump_t found[IG_GJ_MAX];
	int nfound = 0, a, k, i, added = 0;
	float speed = LibVarValue("ig_gapjumpspeed", "300"), maxgap = LibVarValue("ig_gapjumpmax", "260");
	float price = LibVarValue("ig_gapjumpprice", "0.3");
	int debug = (int) LibVarValue("ig_gapjumpdebug", "0");
	/* IG-47: momentum jumps -- longer gaps and drops a run cannot make, at speed */
	int fast = (int) LibVarValue("ig_fastjumps", "0"), nfast = 0;
	float fastmax = LibVarValue("ig_fastjumpmax", "420"), fastdrop = LibVarValue("ig_fastjumpdrop", "160");
	float fastneed = LibVarValue("ig_fastjumpneed", "450"), fastprice = LibVarValue("ig_fastjumpprice", "0.1");

	igfj_enabled = 0;
	if (!LibVarValue("ig_gapjumps", "0") && !fast) return;
	if (debug >= 3) {	/* the areas at points west of the watch point */
		int q;
		for (q = 0; q < 6; q++) {
			vec3_t pt; int pa;
			VectorSet(pt, atof(LibVarString("ig_gapjumpwatchx", "0")) + 40 - q * 40, atof(LibVarString("ig_gapjumpwatchy", "0")), atof(LibVarString("ig_gapjumpwatchz", "376")));
			pa = AAS_PointAreaNum(pt);
			if (pa) botimport.Print(PRT_MESSAGE, "ig: gapjump point %.0f %.0f %.0f area %d mins %.0f %.0f %.0f maxs %.0f %.0f %.0f centre %.0f %.0f grounded %d\n", pt[0], pt[1], pt[2], pa,
				aasworld.areas[pa].mins[0], aasworld.areas[pa].mins[1], aasworld.areas[pa].mins[2], aasworld.areas[pa].maxs[0], aasworld.areas[pa].maxs[1], aasworld.areas[pa].maxs[2],
				aasworld.areas[pa].center[0], aasworld.areas[pa].center[1], AAS_AreaGrounded(pa));
			else botimport.Print(PRT_MESSAGE, "ig: gapjump point %.0f %.0f %.0f no area\n", pt[0], pt[1], pt[2]);
		}
	}
	if (!LibVarValue("ig_gapjumps", "0")) maxgap = 0;	/* momentum jumps only */
	for (a = 1; a < aasworld.numareas && nfound < IG_GJ_MAX; a++) {
		aas_area_t *ar = &aasworld.areas[a];
		vec3_t mid0;
		vec3_t o;

		VectorSet(o, ar->center[0], ar->center[1], ar->mins[2] + 1);
		if (debug >= 3 && ar->mins[0] < atof(LibVarString("ig_gapjumpwatchx", "0")) + 200 && ar->maxs[0] > atof(LibVarString("ig_gapjumpwatchx", "0")) - 200 && ar->mins[1] < atof(LibVarString("ig_gapjumpwatchy", "0")) && ar->maxs[1] > atof(LibVarString("ig_gapjumpwatchy", "0")) && ar->maxs[2] > 300 && ar->mins[2] < 400)
			botimport.Print(PRT_MESSAGE, "ig: gapjump near area %d centre %.0f %.0f %.0f mins %.0f %.0f maxs %.0f %.0f grounded %d crouch %d contents %d inarea %d ground %d\n", a, o[0], o[1], o[2],
				ar->mins[0], ar->mins[1], ar->maxs[0], ar->maxs[1], AAS_AreaGrounded(a), AAS_AreaCrouch(a), aasworld.areasettings[a].contents, AAS_PointAreaNum(o), IGJ_Ground(o, 32, mid0));
		if (!AAS_AreaGrounded(a) || AAS_AreaSwim(a) || AAS_AreaCrouch(a)) continue;
		if (aasworld.areasettings[a].contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_JUMPPAD | AREACONTENTS_TELEPORTER)) continue;
		/* take-off points: the centre, and with ig_fastjumps up to 5 along each
		 * long side -- bspc cuts q2rdm11's walkway end into thin north-south
		 * strips whose centres sit 190 units south of where a bot runs */
		{
		int nsx = 1, nsy = 1, ix, iy;
		if (fast) {
			float ex = ar->maxs[0] - ar->mins[0], ey = ar->maxs[1] - ar->mins[1];
			if (ex > 96) nsx = ex / 64 > 5 ? 5 : (int) (ex / 64);
			if (ey > 96) nsy = ey / 64 > 5 ? 5 : (int) (ey / 64);
		}
		for (ix = 0; ix < nsx; ix++)
		for (iy = 0; iy < nsy; iy++) {
		VectorSet(o, nsx == 1 ? ar->center[0] : ar->mins[0] + (ix + 0.5f) * (ar->maxs[0] - ar->mins[0]) / nsx,
			nsy == 1 ? ar->center[1] : ar->mins[1] + (iy + 0.5f) * (ar->maxs[1] - ar->mins[1]) / nsy, ar->mins[2] + 1);
		if (AAS_PointAreaNum(o) != a || !IGJ_Ground(o, 32, o)) continue;
		for (k = 0; k < IG_GJ_DIRS && nfound < IG_GJ_MAX; k++) {
			vec3_t d, q, n, f, e, L, land, below, mid;
			float s, gap, t, needsp = 0;
			int ta, la, ok = 0;
			aas_trace_t tr;

			VectorSet(d, cos(k * 2 * M_PI / IG_GJ_DIRS), sin(k * 2 * M_PI / IG_GJ_DIRS), 0);
			VectorCopy(o, q);
			for (s = 8; s <= 600; s += 8) {
				VectorMA(q, 8, d, n);
				tr = AAS_TraceClientBBox(q, n, PRESENCE_NORMAL, -1);
				if (tr.startsolid || tr.fraction < 1) break;	/* a wall: no edge this way */
				if (!IGJ_Ground(n, 40, f) || f[2] < q[2] - 20) { ok = 1; break; }	/* the floor ends */
				VectorCopy(f, q);
			}
			if (!ok) continue;
			VectorCopy(q, e);
			ta = AAS_PointAreaNum(e);
			if (!ta || !AAS_AreaGrounded(ta)) { IGJ_Rej(debug, "edge area", e, k); continue; }
			/* the far side: floor at the same height (momentum jumps: further, and
			 * down to ig_fastjumpdrop below) */
			la = 0;
			for (gap = 40; gap <= (fast ? (fastmax > maxgap ? fastmax : maxgap) : maxgap); gap += 8) {
				float drop = fast ? fastdrop : 48;
				VectorMA(e, gap, d, L);
				L[2] += 18;
				if (AAS_PointAreaNum(L) == 0 && (AAS_PointContents(L) & CONTENTS_SOLID)) break;	/* into a wall */
				if (IGJ_Ground(L, 66 + drop - 48, f) && f[2] >= e[2] - drop && f[2] <= e[2] + 18) {
					la = AAS_PointAreaNum(f);
					break;
				}
			}
			if (!la || la == ta || !AAS_AreaGrounded(la)) { IGJ_Rej(debug, la ? "far area" : "no far floor", e, k); continue; }
			if (aasworld.areasettings[la].contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_JUMPPAD | AREACONTENTS_TELEPORTER)) continue;
			if (IGJ_HasRoute(ta, la)) {
				IGJ_Rej(debug, "has route", e, k);
				continue;
			}
			{
				int c1 = aasworld.areasettings[ta].cluster, c2 = aasworld.areasettings[la].cluster;
				if (!(c1 == c2 || c1 < 0 || c2 < 0)) { IGJ_Rej(debug, "clusters", e, k); continue; }
			}
			/* what is under the gap: a floor, not lava, slime or nothing */
			VectorMA(e, gap * 0.5f, d, mid);
			VectorCopy(mid, below); below[2] -= 4000;
			tr = AAS_TraceClientBBox(mid, below, PRESENCE_NORMAL, -1);
			if (tr.fraction >= 1) { IGJ_Rej(debug, "void", e, k); continue; }
			if (AAS_PointContents(tr.endpos) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			{
				vec3_t b2; VectorCopy(tr.endpos, b2); b2[2] -= 8;
				if (AAS_PointContents(b2) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			}
			{	/* the slowest take-off that lands: a run (a TRAVEL_JUMP), or with
				 * ig_fastjumps up to ig_fastjumpneed (a TRAVEL_STRAFEJUMP, offered
				 * only to a bot already moving fast: IG_FastJumpTfl) */
				float sp, top = fast ? fastneed : speed;
				int lowjump;
				for (sp = speed; sp <= top; sp += 25)
					if (IGJ_Fly(e, d, sp, la, land, &t)) break;
				if (sp > top) { IGJ_Rej(debug, "flight", e, k); continue; }
				lowjump = gap <= maxgap && land[2] >= e[2] - 48 && sp <= speed;
				if (!lowjump && !fast) continue;
				needsp = sp;
				/* a momentum jump is only offered from ig_fastjumpon u/s: price its
				 * flight at that speed when that still lands there */
				if (sp > speed) {
					vec3_t land2; float t2, on = LibVarValue("ig_fastjumpon", "500");
					if (on > sp && IGJ_Fly(e, d, on, la, land2, &t2)) t = t2;
				}
			}
			/* one per pair of areas and take-off spot: the narrowest gap (spots
			 * 96+ apart stay apart -- the routing picks by where a bot comes in) */
			for (i = 0; i < nfound; i++)
				if (found[i].from == ta && found[i].r.areanum == la &&
					(!fast || (found[i].r.start[0] - e[0]) * (found[i].r.start[0] - e[0]) + (found[i].r.start[1] - e[1]) * (found[i].r.start[1] - e[1]) < 96 * 96)) break;
			if (i < nfound) { if (found[i].gap <= gap) continue; }
			else i = nfound++;
			Com_Memset(&found[i], 0, sizeof(found[i]));
			found[i].from = ta;
			found[i].gap = gap;
			found[i].r.areanum = la;
			VectorCopy(e, found[i].r.start);
			VectorCopy(land, found[i].r.end);
			if (needsp > speed) {
				found[i].r.traveltype = TRAVEL_STRAFEJUMP;
				found[i].r.traveltime = (unsigned short) (100 * (t + fastprice));
			} else {
				found[i].r.traveltype = TRAVEL_JUMP;
				found[i].r.traveltime = (unsigned short) (100 * (t + price));
			}
			found[i].need = needsp;
			/* a momentum jump is also a route from the area the straight run-up
			 * starts in (the walkway itself): bspc joins thin floor strips only
			 * through points at their middle, so routed via the edge area the run
			 * prices as a dogleg and the long way round wins (q2rdm11's walkway end) */
			if (needsp > speed && a != ta && a != la && nfound < IG_GJ_MAX && !IGJ_HasRoute(a, la)) {
				int c1 = aasworld.areasettings[a].cluster, c2 = aasworld.areasettings[la].cluster;
				vec3_t run; float on = LibVarValue("ig_fastjumpon", "500"), rl;
				VectorSubtract(e, o, run); run[2] = 0; rl = VectorLength(run);
				if ((c1 == c2 || c1 < 0 || c2 < 0) && rl > 32) {
					for (i = 0; i < nfound; i++)
						if (found[i].from == a && found[i].r.areanum == la &&
							(found[i].r.start[0] - o[0]) * (found[i].r.start[0] - o[0]) + (found[i].r.start[1] - o[1]) * (found[i].r.start[1] - o[1]) < 96 * 96) break;
					if (i == nfound) {
						i = nfound++;
						Com_Memset(&found[i], 0, sizeof(found[i]));
						found[i].from = a;
						found[i].gap = gap;
						found[i].need = needsp;
						found[i].r.areanum = la;
						VectorCopy(o, found[i].r.start);
						VectorCopy(land, found[i].r.end);
						found[i].r.traveltype = TRAVEL_STRAFEJUMP;
						found[i].r.traveltime = (unsigned short) (100 * (rl / on + t + fastprice));
					}
				}
			}
		}
		}	/* take-off points */
		}
	}
	/* bspc's own jumps over gaps: priced as a long detour (q2rdm11's walkway gap
	 * 7.3 s, so the route drops into the pit and takes the teleporter back up).
	 * A jump whose flight lands where it says, over a floor that is not lava,
	 * slime or nothing, costs its flight and the take-off price
	 * (ig_gapjumpreprice, default 1 with ig_gapjumps). */
	if (LibVarValue("ig_gapjumps", "0") && LibVarValue("ig_gapjumpreprice", "1")) {
		int r, repriced = 0;
		for (r = 1; r < aasworld.reachabilitysize; r++) {
			aas_reachability_t *rr = &aasworld.reachability[r];
			vec3_t d, land, mid, below;
			float t, dist, sp;
			aas_trace_t tr;
			unsigned short nt;

			if ((rr->traveltype & TRAVELTYPE_MASK) != TRAVEL_JUMP) continue;
			VectorSubtract(rr->end, rr->start, d); d[2] = 0;
			dist = VectorNormalize(d);
			if (dist < 32 || fabs(rr->end[2] - rr->start[2]) > 48) continue;
			VectorMA(rr->start, dist * 0.5f, d, mid); mid[2] = rr->start[2];
			VectorCopy(mid, below); below[2] -= 4000;
			tr = AAS_TraceClientBBox(mid, below, PRESENCE_NORMAL, -1);
			if (tr.fraction >= 1) continue;
			if (AAS_PointContents(tr.endpos) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			VectorCopy(tr.endpos, below); below[2] -= 8;
			if (AAS_PointContents(below) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			for (sp = speed; sp <= speed + 100; sp += 50)
				if (IGJ_Fly(rr->start, d, sp, rr->areanum, land, &t)) break;
			if (sp > speed + 100) continue;
			nt = (unsigned short) (100 * (t + price) + 0.5f);
			if (nt >= rr->traveltime) continue;
			if (debug) botimport.Print(PRT_MESSAGE, "ig: gapjump reprice reach %d -> area %d gap %.0f time %d -> %d from %.0f %.0f %.0f\n", r, rr->areanum, dist, rr->traveltime, nt, rr->start[0], rr->start[1], rr->start[2]);
			rr->traveltime = nt;
			repriced++;
		}
		botimport.Print(PRT_MESSAGE, "ig: %d jumps repriced\n", repriced);
	}
	/* per take-off area: its block of routes moved to the end with the new ones */
	for (i = 0; i < nfound; i++) {
		int from = found[i].from, n = 0, j, newsize;
		aas_reachability_t *nr;
		aas_areasettings_t *as;

		if (from <= 0) continue;
		for (j = i; j < nfound; j++) if (found[j].from == from) n++;
		as = &aasworld.areasettings[from];
		newsize = aasworld.reachabilitysize + as->numreachableareas + n;
		nr = (aas_reachability_t *) GetClearedMemory(newsize * sizeof(aas_reachability_t));
		Com_Memcpy(nr, aasworld.reachability, aasworld.reachabilitysize * sizeof(aas_reachability_t));
		Com_Memcpy(nr + aasworld.reachabilitysize, aasworld.reachability + as->firstreachablearea, as->numreachableareas * sizeof(aas_reachability_t));
		k = aasworld.reachabilitysize + as->numreachableareas;
		for (j = i; j < nfound; j++)
			if (found[j].from == from) {
				nr[k++] = found[j].r;
				if (debug) botimport.Print(PRT_MESSAGE, "ig: %s area %d -> %d gap %.0f need %.0f from %.0f %.0f %.0f to %.0f %.0f %.0f time %d\n",
						(found[j].r.traveltype & TRAVELTYPE_MASK) == TRAVEL_STRAFEJUMP ? "fastjump" : "gapjump", from, found[j].r.areanum,
						found[j].gap, found[j].need, found[j].r.start[0], found[j].r.start[1], found[j].r.start[2], found[j].r.end[0], found[j].r.end[1], found[j].r.end[2], found[j].r.traveltime);
				if ((found[j].r.traveltype & TRAVELTYPE_MASK) == TRAVEL_STRAFEJUMP) nfast++;
				found[j].from = -1;
			}
		as->firstreachablearea = aasworld.reachabilitysize;
		as->numreachableareas += n;
		FreeMemory(aasworld.reachability);
		aasworld.reachability = nr;
		aasworld.reachabilitysize = newsize;
		added += n;
	}
	IG_BlockedResize();
	botimport.Print(PRT_MESSAGE, "ig: %d gap jumps added (%d momentum jumps)\n", added, nfast);
	igfj_enabled = fast;
	igfj_onspd = LibVarValue("ig_fastjumpon", "500");
	igfj_offspd = LibVarValue("ig_fastjumpoff", "380");
	Com_Memset(igfj_on, 0, sizeof(igfj_on));
}
