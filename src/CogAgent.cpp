#include "SteerLib.h"
#include "CogAgent.h"
#include "CogAIModule.h"

#define AGENT_RADIUS 0.4f
#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f
#define COL_QUERY_RADIUS 1.0f

CogAgent::CogAgent()
{
        _enabled = false;
        _radius = AGENT_RADIUS;
}

CogAgent::~CogAgent()
{
        if (_enabled) {
                Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
                gSpatialDatabase->removeObject( this, bounds);
        }
}

void CogAgent::disable()
{
        Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
        gSpatialDatabase->removeObject( this, bounds);
        _enabled = false;
}

void CogAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
        // compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
        // because the value is not used in that case.
        Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

        // initialize the agent based on the initial conditions
        _position = initialConditions.position;
        _forward = initialConditions.direction;
        _radius = initialConditions.radius;
        _velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

        // compute the "new" bounding box of the agent
        Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

        if (!_enabled) {
                // if the agent was not enabled, then it does not already exist in the database, so add it.
                gSpatialDatabase->addObject(this, newBounds);
        }
        else {
                // if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
                gSpatialDatabase->updateObject(this, oldBounds, newBounds);
        }

        _enabled = true;

        if (initialConditions.goals.size() == 0) {
                throw Util::GenericException("No goals were specified!\n");
        }

        // iterate over the sequence of goals specified by the initial conditions.
        for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
                if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
                        _goalQueue.push(initialConditions.goals[i]);
                        if (initialConditions.goals[i].targetIsRandom) {
                                // if the goal is random, we must randomly generate the goal.
                                SteerLib::AgentGoalInfo _goal;
                                _goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
                                _goalQueue.push(_goal);
                        }
                }
                else {
                        throw Util::GenericException("Unsupported goal type; CogAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
                }
        }

        assert(_forward.length()!=0.0f);
        assert(_goalQueue.size() != 0);
        assert(_radius != 0.0f);
}


void CogAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
        // for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
        // the error check for this was performed in reset().
        Util::AutomaticFunctionProfiler profileThisFunction( &CogAIGlobals::gPhaseProfilers->aiProfiler );

        if (!prepareGoalQuery()) return;
        if (!prepareCollisionQuery()) return;

        auto velocity_force = compute_velocity_force();
        auto agent_collision_force = compute_agent_collision_force();
        auto obstacle_collision_force = compute_obstacle_collision_force();
        auto total_force = velocity_force + agent_collision_force + obstacle_collision_force;

        apply_rigid_body_force(total_force, dt);
}

SteerLib::EngineInterface * CogAgent::getSimulationEngine()
{
        return gEngine;
}

void CogAgent::draw()
{
#ifdef ENABLE_GUI
        // if the agent is selected, do some annotations just for demonstration
        if (gEngine->isAgentSelected(this)) {
                Util::Ray ray;
                ray.initWithUnitInterval(_position, _forward);
                float t = 0.0f;
                SteerLib::SpatialDatabaseItem * objectFound;
                Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
                if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
                        Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
                }
                else {
                        Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
                }

                if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
                        Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
                }
        }
        else {
                Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
        }
        CogAlgorithm::draw();
#endif
}


bool CogAgent::prepareGoalQuery()
{
        Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

        // it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
        // for the simple AI, if the agent"s distance to its goal is less than its radius, then the agent has reached the goal.
        if (vectorToGoal.lengthSquared() < _radius * _radius) {
                _goalQueue.pop();
                if (_goalQueue.size() != 0) {
                        // in this case, there are still more goals, so start steering to the next goal.
                        vectorToGoal = _goalQueue.front().targetLocation - _position;
                }
                else {
                        // in this case, there are no more goals, so disable the agent and remove it from the spatial database.
                        disable();
                        return false;
                }
        }

        return true;
}

bool CogAgent::prepareCollisionQuery()
{
        _collisionAgents.clear();
        _collisionObstacles.clear();
        Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
        std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
        getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
                        _position.x-(this->_radius + COL_QUERY_RADIUS),
                        _position.x+(this->_radius + COL_QUERY_RADIUS),
                        _position.z-(this->_radius + COL_QUERY_RADIUS),
                        _position.z+(this->_radius + COL_QUERY_RADIUS),
                        (this));

        SteerLib::AgentInterface * agent;
        SteerLib::ObstacleInterface * obstacle;

        for (auto &it : _neighbors) {
                agent = dynamic_cast<SteerLib::AgentInterface *>(it);
                if (agent != NULL) {
                        _collisionAgents.push_back(agent);
                        continue;
                }
                obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(it);
                if (obstacle != NULL) {
                        _collisionObstacles.push_back(obstacle);
                        continue;
                }
                std::cerr << "Warning: Unknown object type in spatial database.\n";
        }

        return true;
}
