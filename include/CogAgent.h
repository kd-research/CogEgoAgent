// @author Kaidong Hu

#pragma once

#include "CogAIModule.h"
#include "CogAlgorithm.h"
#include "SteerLib.h"
#include <queue>

class CogAgent : public CogAlgorithm
{
  public:
    CogAgent();
    ~CogAgent();
    void reset(const SteerLib::AgentInitialConditions &initialConditions, SteerLib::EngineInterface *engineInfo);
    void updateAI(float timeStamp, float dt, unsigned int frameNumber);
    void disable();
    void draw();

    bool enabled() const { return _enabled; }
    Util::Point position() const { return _position; }
    Util::Vector forward() const { return _forward; }
    Util::Vector velocity() const { return _velocity; }
    float radius() const { return _radius; }
    size_t id() const { return 0; }

    const SteerLib::AgentGoalInfo &currentGoal() const { return _goalQueue.front(); }
    const std::queue<SteerLib::AgentGoalInfo> &agentGoals() const
    {
        throw Util::GenericException("agentGoals() not implemented yet");
    }
    void addGoal(const SteerLib::AgentGoalInfo &newGoal)
    {
        throw Util::GenericException("addGoals() not implemented yet for CogAgent");
    }
    void clearGoals() { throw Util::GenericException("clearGoals() is not implemented yet for CogAgent"); }

    void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq)
    {
        throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent");
    }
    void setParameters(SteerLib::Behaviour behave)
    {
        throw Util::GenericException("setParameters() not implemented yet for this Agent");
    }

    /// @name The SteerLib::SpatialDatabaseItemInterface
    /// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial
    /// database; The Util namespace helper functions do the job nicely for basic circular agents.
    //@{
    bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
    bool overlaps(const Util::Point &p, float radius)
    {
        return Util::circleOverlapsCircle2D(_position, _radius, p, radius);
    }
    float computePenetration(const Util::Point &p, float radius)
    {
        return Util::computeCircleCirclePenetration2D(_position, _radius, p, radius);
    }
    //@}

  protected:
    virtual SteerLib::EngineInterface *getSimulationEngine();
    bool prepareCollisionQuery();
    bool prepareGoalQuery();

  private:
    bool disable_when_goal_reached();
};
