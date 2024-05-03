#pragma once

#include "CogConfig.h"
#include "SteerLib.h"

class CogAlgorithm : virtual public SteerLib::AgentInterface
{
public:
	CogAlgorithm() { _config = CogConfig::getInstance(); }
	Util::Vector compute_velocity_force();
	Util::Vector compute_agent_collision_force();
	Util::Vector compute_obstacle_collision_force();

	void apply_rigid_body_force(const Util::Vector & force, float dt);
	void draw();
protected:
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	std::vector<SteerLib::AgentInterface *> _collisionAgents;
	std::vector<SteerLib::ObstacleInterface *> _collisionObstacles;
	CogConfig *_config;
};
		
