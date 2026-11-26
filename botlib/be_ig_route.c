/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_route.c -- IG-46: momentum-aware route times (libvar ig_routemomentum).
 *
 * The routing prices walking across an area at 300 u/s whatever the path:
 * AAS_CalculateAreaTravelTimes fills, per area, the time from where each
 * route in ends to where each route out starts, as distance x 0.33. A bot
 * that strafe-hops (or a player) crosses a long straight line at 500-700 u/s
 * and slows only to turn, so q2rdm11's walkway, ~2250 units, is priced 7.4 s
 * and hopped in ~3.5 s, and the routing takes the teleporter instead (Billy's
 * line from the pad: priced 11.2 s, flown in 5-6; the teleporter route 7.1 s).
 *
 * Here each crossing is priced by how straight it is: the turn from the route
 * in to the crossing and from the crossing to the route out, the larger of
 * the two. Up to ig_routestraight (20) degrees: x ig_routefast (0.6, ~500
 * u/s); from ig_routeturn (70) degrees on: unchanged; between: linear. Only
 * on plain floor areas (not crouch, swim, lava or slime), and not after a
 * route in that leaves the bot slow (teleporter, ladder, elevator, train,
 * water jump). Applied once, after the table is built at map load.
 */

#include "../game_q3/q_shared.h"
#include "l_memory.h"
#include "l_libvar.h"
#include "aasfile.h"
#include "../game_q3/botlib.h"
#include "../game_q3/be_aas.h"
#include "be_aas_funcs.h"
#include "be_interface.h"
#include "be_aas_def.h"

static float IGR_Angle(const vec3_t a, const vec3_t b)
{
	float la = sqrt(a[0] * a[0] + a[1] * a[1]), lb = sqrt(b[0] * b[0] + b[1] * b[1]), c;

	if (la < 1 || lb < 1) return 0;
	c = (a[0] * b[0] + a[1] * b[1]) / (la * lb);
	if (c > 1) c = 1;
	if (c < -1) c = -1;
	return acos(c) * 180 / M_PI;
}

void IG_RouteMomentum(void)
{
	int i, l, n, changed = 0, total = 0;
	float fast, straight, turn;
	aas_reversedreachability_t *revreach;
	aas_reversedlink_t *revlink;
	aas_areasettings_t *settings;

	if (!LibVarValue("ig_routemomentum", "0") || !aasworld.areatraveltimes) return;
	fast = LibVarValue("ig_routefast", "0.6");
	straight = LibVarValue("ig_routestraight", "20");
	turn = LibVarValue("ig_routeturn", "70");
	if (turn <= straight) turn = straight + 1;
	for (i = 1; i < aasworld.numareas; i++)
	{
		if (AAS_AreaCrouch(i) || AAS_AreaSwim(i) || AAS_AreaLava(i) || AAS_AreaSlime(i)) continue;
		revreach = &aasworld.reversedreachability[i];
		settings = &aasworld.areasettings[i];
		for (l = 0; l < settings->numreachableareas; l++)
		{
			aas_reachability_t *out = &aasworld.reachability[settings->firstreachablearea + l];
			vec3_t dout;

			/* a walk route's own start and end sit either side of an area edge, a few
			 * units apart: its direction is noise, so the route out points from its
			 * start to the middle of the area it leads to */
			if ((out->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK) VectorSubtract(aasworld.areas[out->areanum].center, out->start, dout);
			else VectorSubtract(out->end, out->start, dout);
			for (n = 0, revlink = revreach->first; revlink; revlink = revlink->next, n++)
			{
				aas_reachability_t *in = &aasworld.reachability[revlink->linknum];
				int tt = in->traveltype & TRAVELTYPE_MASK, t;
				vec3_t din, seg;
				float a, f;

				total++;
				if (tt == TRAVEL_TELEPORT || tt == TRAVEL_LADDER || tt == TRAVEL_ELEVATOR || tt == TRAVEL_FUNCBOB ||
					tt == TRAVEL_FUNCTRAIN || tt == TRAVEL_WATERJUMP) continue;
				if (tt == TRAVEL_WALK) VectorSubtract(in->end, aasworld.areas[revlink->areanum].center, din);
				else VectorSubtract(in->end, in->start, din);
				VectorSubtract(out->start, in->end, seg);
				a = IGR_Angle(din, seg);
				f = IGR_Angle(seg, dout);
				if (f > a) a = f;
				if (a >= turn) continue;
				f = a <= straight ? fast : fast + (1 - fast) * (a - straight) / (turn - straight);
				t = (int) (aasworld.areatraveltimes[i][l][n] * f);
				if (t < 1) t = 1;
				if (t != aasworld.areatraveltimes[i][l][n]) changed++;
				aasworld.areatraveltimes[i][l][n] = (unsigned short) t;
			}
		}
	}
	if (LibVarValue("ig_routemomentumdebug", "0"))
		botimport.Print(PRT_MESSAGE, "ig: routemomentum %d of %d crossings priced faster\n", changed, total);
}

/* debug: the route from a point to a goal area, each step's crossing angles and time */
void IG_RouteDebugPath(const char *tag, vec3_t org, int goal)
{
	int a = AAS_PointAreaNum(org), k, tt, rn, prev = 0;
	vec3_t o;

	VectorCopy(org, o);
	for (k = 0; k < 60 && a && a != goal; k++) {
		aas_reachability_t *out;
		vec3_t din, seg, dout;
		float a1 = -1, a2;
		if (!AAS_AreaRouteToGoalArea(a, o, goal, TFL_DEFAULT | TFL_GRAPPLEHOOK, &tt, &rn) || !rn) break;
		out = &aasworld.reachability[rn];
		if ((out->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK) VectorSubtract(aasworld.areas[out->areanum].center, out->start, dout);
		else VectorSubtract(out->end, out->start, dout);
		VectorSubtract(out->start, o, seg);
		if (prev) {
			aas_reachability_t *in = &aasworld.reachability[prev];
			if ((in->traveltype & TRAVELTYPE_MASK) == TRAVEL_WALK) {
				int src = 0, i;
				/* the area the route in came from */
				for (i = 1; i < aasworld.numareas && !src; i++)
					if (rn && prev >= aasworld.areasettings[i].firstreachablearea && prev < aasworld.areasettings[i].firstreachablearea + aasworld.areasettings[i].numreachableareas) src = i;
				VectorSubtract(in->end, aasworld.areas[src].center, din);
			}
			else VectorSubtract(in->end, in->start, din);
			a1 = IGR_Angle(din, seg);
		}
		a2 = IGR_Angle(seg, dout);
		botimport.Print(PRT_MESSAGE, "igrdbg %s %d area %d type %d seg %.0f in %.0f out %.0f left %d at %.0f %.0f %.0f\n", tag, k, a,
			out->traveltype & TRAVELTYPE_MASK, VectorLength(seg), a1, a2, tt, out->start[0], out->start[1], out->start[2]);
		VectorCopy(out->end, o);
		a = out->areanum; prev = rn;
	}
}
