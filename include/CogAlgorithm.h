#pragma once

#include "SteerLib.h"

class CogAlgorithm : virtual public SteerLib::AgentInterface
{
public:
	Util::Vector compute_velocity_force();
	Util::Vector compute_agent_collision_force();
	Util::Vector compute_obstacle_collision_force();

	void check_neighbors();
	void apply_rigid_body_force(const Util::Vector & force, float dt);

protected:
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	std::vector<SteerLib::AgentInterface *> _collisionAgents;
	std::vector<SteerLib::ObstacleInterface *> _collisionObstacles;
};
		
