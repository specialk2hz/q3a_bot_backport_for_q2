/*
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
*/
/* be_ig_links.c -- IG-49: measured link times (libvar ig_links, default 0).
 *
 * Billy (2026-11-28): "I dont need to train every single jump the logic
 * should see the opportunity and take it if it makes sense". The routing
 * prices every step at walking speed (300 u/s), so a fast line -- a walkway
 * the bots hop at 800 u/s, a momentum jump -- looks no better than the long
 * way round and routes seldom pass it (momentum jumps taken in fights: 0.02
 * per bot-minute). Here the routing learns from the bots themselves:
 *
 *   ig_linklog 1   every move a bot makes from one area into the next logs
 *                  "igl <from> <to> <ms in from>" (igbench roaming runs);
 *                  tools/linktable.py turns the logs into maps/<map>.links
 *   ig_links 1     at map load, after the area travel times: for each logged
 *                  pair with enough samples, the route out of <from> into
 *                  <to> costs the measured time (its crossing times zeroed),
 *                  clamped to ig_linkmin..ig_linkmax (0.3..1.5) of the
 *                  walking estimate */

#include "../game_q3/q_shared.h"
#include "l_memory.h"
#include "l_libvar.h"
#include "aasfile.h"
#include "../game_q3/botlib.h"
#include "../game_q3/be_aas.h"
#include "be_aas_funcs.h"
#include "be_interface.h"
#include "be_aas_def.h"

static int igl_area[MAX_CLIENTS];
/* per area: how fast the bots really cross it against the walking estimate
 * (measured / estimated, ig_linkmin..ig_linkmax); 1 where nothing was measured.
 * One factor scales everything inside an area -- its crossings and the walk
 * from where a bot stands to its next route -- so a neighbour never looks
 * nearer the goal than the area itself (pricing single routes did that: 28
 * maps, frags -0.35/min, -8 u/s) */
static float *igl_factor;
static int igl_numareas;
float IG_LinkAreaFactor(int area)
{
	if (!igl_factor || area <= 0 || area >= igl_numareas) return 1;
	return igl_factor[area];
}
static float igl_t0[MAX_CLIENTS];
static vec3_t igl_o0[MAX_CLIENTS];

void IG_LinkObserve(int client, const vec3_t origin)
{
	int a;
	float now;
	vec3_t d;

	if (client < 0 || client >= MAX_CLIENTS || !LibVarValue("ig_linklog", "0")) return;
	a = AAS_PointAreaNum((vec_t *) origin);
	if (!a || a == igl_area[client]) return;
	now = AAS_Time();
	VectorSubtract(origin, igl_o0[client], d);
	if (igl_area[client] && now > igl_t0[client] && now - igl_t0[client] < 3 && VectorLength(d) < 600)
		botimport.Print(PRT_MESSAGE, "igl %d %d %d\n", igl_area[client], a, (int) ((now - igl_t0[client]) * 1000 + 0.5f));
	igl_area[client] = a;
	igl_t0[client] = now;
	VectorCopy(origin, igl_o0[client]);
}

void IG_LinksAtLoad(void)
{
	char path[MAX_QPATH], *buf, *line, *next;
	fileHandle_t fp;
	int len, applied = 0, lines = 0, size = -1;
	float lo = LibVarValue("ig_linkmin", "0.3"), hi = LibVarValue("ig_linkmax", "1.5");
	int minn = (int) LibVarValue("ig_linksamples", "4");

	Com_Memset(igl_area, 0, sizeof(igl_area));
	if (LibVarValue("ig_linklog", "0")) botimport.Print(PRT_MESSAGE, "igl areas %d map %s\n", aasworld.numareas, aasworld.mapname);
	if (!LibVarValue("ig_links", "0") || !aasworld.areatraveltimes) return;
	Com_sprintf(path, sizeof(path), "maps/%s.links", aasworld.mapname);
	len = botimport.FS_FOpenFile(path, &fp, FS_READ);
	if (!fp || len <= 0) { botimport.Print(PRT_MESSAGE, "ig: no link table %s\n", path); return; }
	buf = calloc(len + 1, 1);
	if (!buf) { botimport.FS_FCloseFile(fp); return; }
	botimport.FS_Read(buf, len, fp);
	botimport.FS_FCloseFile(fp);
	free(igl_factor); igl_factor = NULL; igl_numareas = 0;
	{
		float *sum = calloc(aasworld.numareas, sizeof(float));
		int *cnt = calloc(aasworld.numareas, sizeof(int)), a2, i, j;
		for (line = buf; line && *line; line = next) {
			int from, to, ms, n, l, k;
			aas_areasettings_t *s2;
			next = strchr(line, '\n');
			if (next) *next++ = 0;
			if (sscanf(line, "areas %d", &size) == 1) {
				if (size != aasworld.numareas) { botimport.Print(PRT_MESSAGE, "ig: link table %s is for another .aas (%d areas, this %d)\n", path, size, aasworld.numareas); break; }
				continue;
			}
			if (sscanf(line, "%d %d %d %d", &from, &to, &ms, &n) != 4) continue;
			lines++;
			if (n < minn || from <= 0 || from >= aasworld.numareas || to <= 0 || to >= aasworld.numareas) continue;
			s2 = &aasworld.areasettings[from];
			for (l = 0; l < s2->numreachableareas; l++) {
				aas_reachability_t *r = &aasworld.reachability[s2->firstreachablearea + l];
				aas_reversedreachability_t *rev = &aasworld.reversedreachability[from];
				float def = 0; int c2 = 0;
				if (r->areanum != to || (r->traveltype & TRAVELTYPE_MASK) != TRAVEL_WALK) continue;
				for (k = 0; k < rev->numlinks; k++) { def += aasworld.areatraveltimes[from][l][k]; c2++; }
				def = (c2 ? def / c2 : 0) + r->traveltime;
				if (def < 3) continue;		/* too short to say */
				sum[from] += (ms / 10.0f) / def; cnt[from]++;
			}
		}
		igl_factor = malloc(aasworld.numareas * sizeof(float));
		igl_numareas = aasworld.numareas;
		for (a2 = 0; a2 < aasworld.numareas; a2++) {
			float f = cnt[a2] ? sum[a2] / cnt[a2] : 1;
			if (f < lo) f = lo;
			if (f > hi) f = hi;
			igl_factor[a2] = f;
			if (cnt[a2] && f != 1) {
				aas_areasettings_t *s3 = &aasworld.areasettings[a2];
				aas_reversedreachability_t *rev = &aasworld.reversedreachability[a2];
				for (i = 0; i < s3->numreachableareas; i++)
					for (j = 0; j < rev->numlinks; j++)
						aasworld.areatraveltimes[a2][i][j] = (unsigned short) (aasworld.areatraveltimes[a2][i][j] * f + 0.5f);
				applied++;
			}
		}
		free(sum); free(cnt);
	}
	free(buf);
	botimport.Print(PRT_MESSAGE, "ig: link table %s: %d pairs, %d areas priced as measured\n", path, lines, applied);
}
