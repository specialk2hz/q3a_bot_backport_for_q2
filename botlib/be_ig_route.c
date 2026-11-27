/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_route.c -- a route debug print for the pad drill (IG-43): the route's steps
 * from a point to a goal area, each with its crossing angles and time. (IG-46's
 * route momentum lived here; tried twice with no gain, removed 2026-11-28.) */

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
		if (!AAS_AreaRouteToGoalArea(a, o, goal, TFL_DEFAULT | TFL_GRAPPLEHOOK | (LibVarValue("ig_routedebugfast", "0") ? TFL_STRAFEJUMP : 0), &tt, &rn) || !rn) break;
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
