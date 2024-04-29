// @author: Kaidong Hu
#pragma once

#include "SteerLib.h"
#include "Logger.h"
#include "CogConfig.h"

extern SteerLib::EngineInterface * gEngine;
extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

namespace CogAIGlobals {

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};


	extern unsigned int gLongTermPlanningPhaseInterval;
	extern unsigned int gMidTermPlanningPhaseInterval;
	extern unsigned int gShortTermPlanningPhaseInterval;
	extern unsigned int gPredictivePhaseInterval;
	extern unsigned int gReactivePhaseInterval;
	extern unsigned int gPerceptivePhaseInterval;
	extern bool gUseDynamicPhaseScheduling;
	extern bool gShowStats;
	extern bool gShowAllStats;

	extern PhaseProfilers * gPhaseProfilers;
}



class CogAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void preprocessSimulation();
	void initializeSimulation();
	void cleanupSimulation();
	void draw() override;

protected:
	std::string logFilename;
	bool logStats;
	Logger * _logger;

private:
	CogConfig * _config;
};
