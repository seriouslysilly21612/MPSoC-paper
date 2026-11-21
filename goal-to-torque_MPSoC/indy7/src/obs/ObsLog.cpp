/*****************************************************************************
*	Name: ObsLog.cpp
*	Description: The format of the log of the camera results. See ObsLog.h.
*****************************************************************************/
#include "obs/ObsLog.h"

namespace {

/* the keys of the .json */
const char* const s_apcWorkspace[CGoalFilter::eVerdicts] = { "accepted", "invalid", "not_finite", "below_floor", "outside_box" };
const char* const s_apcWriter[CObsWriter::eResults] = { "written", "invalid", "not_finite", "age", "full", "expired", "clock" };

/* a time of the log [s]: from the start of the run, negative before it */
double Rel(uint64_t atT, const ObsFormat::Meta& astM)
{
	return (atT >= astM.tStart ? (double)(atT - astM.tStart) : -(double)(astM.tStart - atT)) / astM.stSet.ticks_per_s;
}

} // namespace

void
ObsFormat::Name(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen)
{
	snprintf(acOut, auLen, "obs_%s_%s_r%02d_%s", astM.acMethod, astM.bSim ? "sim" : "robot", astM.nRun, astrStamp);
}

void
ObsFormat::Header(FILE* fp, const Meta&)
{
	fprintf(fp, "t,verdict,n,epoch,target,cls,mode,flags,t_e,f,t_valid,t_in,sigma,ebar,s,z_x,z_y,z_z,"
	            "obj_x,obj_y,obj_z,ws,folded\n");
}

void
ObsFormat::Row(FILE* fp, const Meta& astM, const Sample& s)
{
	const obs_held_t& h = s.stRec.h;
	const obs_desc_t& d = h.d;
	fprintf(fp, "%.6f,%s,%u,%u,%u,%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f", Rel(s.stRec.t_out, astM),
	        CObsGoal::VerdictName(s.stRec.verdict), (unsigned)d.n, (unsigned)d.epoch, (unsigned)d.target_seq,
	        (unsigned)d.cls, (unsigned)d.mode, (unsigned)d.flags, Rel(d.t_e, astM), Rel(d.f, astM),
	        Rel(d.t_valid_until, astM), Rel(h.t_in, astM), (double)d.sigma, (double)h.ebar, (unsigned)d.s,
	        (double)d.z[0], (double)d.z[1], (double)d.z[2]);
	if (s.nWorkspace >= 0)
		fprintf(fp, ",%.6f,%.6f,%.6f,%s,%d\n", s.adObj[0], s.adObj[1], s.adObj[2],
		        CGoalFilter::Name((CGoalFilter::eVerdict)s.nWorkspace), s.nFolded);
	else
		fputs(",,,,,0\n", fp);
}

void
ObsFormat::Finish(FILE* fp, const char* astrCsv, const Meta& m, const Result& r, const char* astrState, long anRows,
                  unsigned anDrops)
{
	const char* pcBase = strrchr(astrCsv, '/');
	pcBase = pcBase ? pcBase + 1 : astrCsv;
	const CObsGoal::Counts& c = r.stCounts;
	if (fp != NULL)
	{
		fprintf(fp, "{\n");
		fprintf(fp, "  \"format\": \"g2t-obs-log 1\",\n");
		fprintf(fp, "  \"csv\": \"%s\",\n", pcBase);
		fprintf(fp, "  \"method\": \"%s\",\n  \"run\": %d,\n", m.acMethod, m.nRun);
		fprintf(fp, "  \"platform\": \"%s\",\n", m.bSim ? "simulation" : "robot");
		fprintf(fp, "  \"policy\": \"%s\",\n", m.stSet.policy == OBS_CONTRACT ? "contract" : "max_age");
		fprintf(fp, "  \"ticks_per_s\": %.9g,\n  \"t_start_ticks\": %llu,\n  \"t_start_monotonic_ns\": %llu,\n",
		        m.stSet.ticks_per_s, (unsigned long long)m.tStart, (unsigned long long)m.tStartMonotonicNs);
		fprintf(fp, "  \"period\": %.9g,\n  \"age_cap\": %.9g,\n", (double)m.stSet.period_ticks / m.stSet.ticks_per_s,
		        (double)m.stSet.age_cap_ticks / m.stSet.ticks_per_s);
		fprintf(fp, "  \"delta\": %.6g,\n  \"e0\": %.6g,\n  \"g\": %.6g,\n  \"eps_v\": %.6g,\n  \"a_max\": %.6g,\n"
		        "  \"eps_cal\": %.6g,\n", m.dTolM, m.stSet.e0, m.stSet.g, m.stSet.eps_v, m.stSet.a_max, m.stSet.eps_cal);
		fprintf(fp, "  \"state_age_periods\": %d,\n", m.nStateAge);
		testlog::PrintVec(fp, "tag_offset", m.adTagOffset, 3, "%.9g");
		fprintf(fp, "  \"result\": \"%s\",\n", astrState);
		fprintf(fp, "  \"rows\": %ld,\n  \"dropped\": %u,\n", anRows, anDrops);
		fprintf(fp, "  \"taken\": %u,\n", (unsigned)c.uTaken);
		for (int v = 0; v < (int)OBS_VERDICTS; v++)
			fprintf(fp, "  \"%s\": %u,\n", CObsGoal::VerdictName(v), (unsigned)c.auVerdict[v]);
		fprintf(fp, "  \"workspace\": {");
		for (int v = 0; v < (int)CGoalFilter::eVerdicts; v++)
			fprintf(fp, "%s\"%s\": %ld", v ? ", " : "", s_apcWorkspace[v], c.anWorkspace[v]);
		fprintf(fp, "},\n");
		fprintf(fp, "  \"folded\": %ld,\n  \"restarts\": %ld,\n  \"goals\": %ld,\n", c.nFolded, c.nRestarts, c.nGoals);
		fprintf(fp, "  \"camera_side\": {");
		for (int w = 0; w < (int)CObsWriter::eResults; w++)
			fprintf(fp, "%s\"%s\": %ld", w ? ", " : "", s_apcWriter[w], r.anWriter[w]);
		fprintf(fp, "}\n}\n");
	}
	printf("[OBS] saved %s (%s, run %d: %ld rows, %u dropped)\n", astrCsv, m.acMethod, m.nRun, anRows, anDrops);
	printf("[OBS] %u results taken:", (unsigned)c.uTaken);
	for (int v = 0; v < (int)OBS_VERDICTS; v++)
		if (c.auVerdict[v] != 0)
			printf(" %u %s,", (unsigned)c.auVerdict[v], CObsGoal::VerdictName(v));
	long nNotWritten = 0;
	for (int w = 1; w < (int)CObsWriter::eResults; w++)
		nNotWritten += r.anWriter[w];
	printf(" %ld folded into the estimate (%ld restarts); camera side: %ld written, %ld not written\n",
	       c.nFolded, c.nRestarts, r.anWriter[CObsWriter::eWritten], nNotWritten);
}
