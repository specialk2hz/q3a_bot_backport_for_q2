/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_roamgoal.c -- IG-44: a fair random roam goal (libvar ig_roamgoal).
 *
 * With no item to fetch (instagib: always), BotChooseLTGItem falls back to
 * AAS_RandomGoalArea. Stock picks a random area INDEX and takes the first
 * acceptable area from there, and accepts any swimming area outright while a
 * floor area must pass a ground check. Two biases: water is a goal far more
 * often than its size (q2dm1/q2dm1pro: bots gather in the pool room), and an
 * area right after a long run of rejected ones is picked often.
 *
 * Here: once per map, list the floor areas (ground face over 300, the trace
 * down lands in the area, no water, lava or slime), weighted by floor size,
 * so the goal is a uniform spot on the map's floor. Each call picks among the
 * ones reachable from the bot, at least IG_ROAM_MIN away.
 */

#include "../game_q3/q_shared.h"
#include "l_memory.h"
#include "l_libvar.h"
#include "l_log.h"
#include "aasfile.h"
#include "../game_q3/botlib.h"
#include "../game_q3/be_aas.h"
#include "be_aas_funcs.h"
#include "be_interface.h"
#include "be_aas_def.h"

void AAS_MarkReachableAreas(int areanum, int travelflags, byte *reached, int *queue);	/* be_aas_route.c */

#define IG_ROAM_MIN 256

static char   igr_map[MAX_QPATH];
static int    igr_numareas = -1;
static int    igr_n;
static int   *igr_area;
static float *igr_weight;
static vec3_t *igr_origin;

static void IGR_Build(void)
{
	int i;
	aas_trace_t trace;
	vec3_t start, end;

	if (igr_area) { FreeMemory(igr_area); FreeMemory(igr_weight); FreeMemory(igr_origin); }
	igr_area = (int *) GetClearedMemory(aasworld.numareas * sizeof(int));
	igr_weight = (float *) GetClearedMemory(aasworld.numareas * sizeof(float));
	igr_origin = (vec3_t *) GetClearedMemory(aasworld.numareas * sizeof(vec3_t));
	igr_n = 0;
	for (i = 1; i < aasworld.numareas; i++)
	{
		float g;
		if (!AAS_AreaReachability(i)) continue;
		if (AAS_AreaSwim(i) || AAS_AreaLava(i) || AAS_AreaSlime(i) || AAS_AreaDoNotEnter(i)) continue;
		g = AAS_AreaGroundFaceArea(i);
		if (g <= 300) continue;
		VectorCopy(aasworld.areas[i].center, start);
		VectorCopy(start, end);
		end[2] -= 300;
		trace = AAS_TraceClientBBox(start, end, PRESENCE_CROUCH, -1);
		if (trace.startsolid || trace.fraction >= 1 || AAS_PointAreaNum(trace.endpos) != i) continue;
		if (AAS_PointContents(trace.endpos) & (CONTENTS_WATER | CONTENTS_LAVA | CONTENTS_SLIME)) continue;
		igr_area[igr_n] = i;
		igr_weight[igr_n] = g;
		VectorCopy(trace.endpos, igr_origin[igr_n]);
		igr_n++;
	}
	Q_strncpyz(igr_map, aasworld.mapname, sizeof(igr_map));
	igr_numareas = aasworld.numareas;
	if (LibVarValue("ig_roamgoaldebug", "0"))
		botimport.Print(PRT_MESSAGE, "ig: roamgoal %s %d floor areas of %d\n", igr_map, igr_n, aasworld.numareas);
}

int IG_RandomGoalArea(int areanum, int travelflags, int *goalareanum, vec3_t goalorigin)
{
	int i, try, *queue, found = 0;
	byte *reached;
	float total;

	if (!AAS_AreaReachability(areanum)) return 0;
	if (igr_numareas != aasworld.numareas || Q_stricmp(igr_map, aasworld.mapname)) IGR_Build();
	if (!igr_n) return 0;
	queue = (int *) GetClearedMemory(aasworld.numareas * (sizeof(int) + sizeof(byte)));
	reached = (byte *) (queue + aasworld.numareas);
	AAS_MarkReachableAreas(areanum, travelflags, reached, queue);
	total = 0;
	for (i = 0; i < igr_n; i++)
		if (reached[igr_area[i]] && igr_area[i] != areanum) total += igr_weight[i];
	for (try = 0; try < 8 && total > 0 && !found; try++)
	{
		float r = random() * total;
		vec3_t d;
		for (i = 0; i < igr_n; i++)
		{
			if (!reached[igr_area[i]] || igr_area[i] == areanum) continue;
			r -= igr_weight[i];
			if (r <= 0) break;
		}
		if (i >= igr_n) i = igr_n - 1;
		VectorSubtract(igr_origin[i], aasworld.areas[areanum].center, d);
		if (VectorLength(d) < IG_ROAM_MIN) continue;
		if (AAS_AreaTravelTimeToGoalArea(areanum, aasworld.areas[areanum].center, igr_area[i], travelflags) <= 0) continue;
		*goalareanum = igr_area[i];
		VectorCopy(igr_origin[i], goalorigin);
		found = 1;
	}
	FreeMemory(queue);
	return found;
}
