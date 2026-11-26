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

typedef struct { int from; aas_reachability_t r; float gap; } igjump_t;

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

void IG_GapJumpsAtLoad(void)
{
	static igjump_t found[IG_GJ_MAX];
	int nfound = 0, a, k, i, added = 0;
	float speed = LibVarValue("ig_gapjumpspeed", "300"), maxgap = LibVarValue("ig_gapjumpmax", "260");
	float price = LibVarValue("ig_gapjumpprice", "0.3");
	int debug = (int) LibVarValue("ig_gapjumpdebug", "0");

	if (!LibVarValue("ig_gapjumps", "0")) return;
	for (a = 1; a < aasworld.numareas && nfound < IG_GJ_MAX; a++) {
		aas_area_t *ar = &aasworld.areas[a];
		vec3_t o;

		if (!AAS_AreaGrounded(a) || AAS_AreaSwim(a) || AAS_AreaCrouch(a)) continue;
		if (aasworld.areasettings[a].contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_JUMPPAD | AREACONTENTS_TELEPORTER)) continue;
		VectorSet(o, ar->center[0], ar->center[1], ar->mins[2] + 1);
		if (AAS_PointAreaNum(o) != a || !IGJ_Ground(o, 32, o)) continue;
		for (k = 0; k < IG_GJ_DIRS && nfound < IG_GJ_MAX; k++) {
			vec3_t d, q, n, f, e, L, land, below, mid;
			float s, gap, t;
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
			if (!ta || !AAS_AreaGrounded(ta)) continue;
			/* the far side: floor at the same height */
			la = 0;
			for (gap = 40; gap <= maxgap; gap += 8) {
				VectorMA(e, gap, d, L);
				L[2] += 18;
				if (AAS_PointAreaNum(L) == 0 && (AAS_PointContents(L) & CONTENTS_SOLID)) break;	/* into a wall */
				if (IGJ_Ground(L, 66, f) && f[2] >= e[2] - 48 && f[2] <= e[2] + 18) {
					la = AAS_PointAreaNum(f);
					break;
				}
			}
			if (!la || la == ta || !AAS_AreaGrounded(la)) continue;
			if (aasworld.areasettings[la].contents & (AREACONTENTS_LAVA | AREACONTENTS_SLIME | AREACONTENTS_JUMPPAD | AREACONTENTS_TELEPORTER)) continue;
			if (IGJ_HasRoute(ta, la)) continue;
			{
				int c1 = aasworld.areasettings[ta].cluster, c2 = aasworld.areasettings[la].cluster;
				if (!(c1 == c2 || c1 < 0 || c2 < 0)) continue;
			}
			/* what is under the gap: a floor, not lava, slime or nothing */
			VectorMA(e, gap * 0.5f, d, mid);
			VectorCopy(mid, below); below[2] -= 4000;
			tr = AAS_TraceClientBBox(mid, below, PRESENCE_NORMAL, -1);
			if (tr.fraction >= 1) continue;
			if (AAS_PointContents(tr.endpos) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			{
				vec3_t b2; VectorCopy(tr.endpos, b2); b2[2] -= 8;
				if (AAS_PointContents(b2) & (CONTENTS_LAVA | CONTENTS_SLIME)) continue;
			}
			if (!IGJ_Fly(e, d, speed, la, land, &t)) continue;
			/* one per pair of areas: the narrowest gap */
			for (i = 0; i < nfound; i++)
				if (found[i].from == ta && found[i].r.areanum == la) break;
			if (i < nfound) { if (found[i].gap <= gap) continue; }
			else i = nfound++;
			Com_Memset(&found[i], 0, sizeof(found[i]));
			found[i].from = ta;
			found[i].gap = gap;
			found[i].r.areanum = la;
			VectorCopy(e, found[i].r.start);
			VectorCopy(land, found[i].r.end);
			found[i].r.traveltype = TRAVEL_JUMP;
			found[i].r.traveltime = (unsigned short) (100 * (t + price));
		}
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
				if (debug) botimport.Print(PRT_MESSAGE, "ig: gapjump area %d -> %d gap %.0f from %.0f %.0f %.0f to %.0f %.0f %.0f time %d\n", from, found[j].r.areanum,
						found[j].gap, found[j].r.start[0], found[j].r.start[1], found[j].r.start[2], found[j].r.end[0], found[j].r.end[1], found[j].r.end[2], found[j].r.traveltime);
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
	botimport.Print(PRT_MESSAGE, "ig: %d gap jumps added\n", added);
}
