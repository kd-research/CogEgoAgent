#include "CogAlgorithm.h"

extern SteerLib::EngineInterface * gEngine;
extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

#define AGENT_MASS 1.0f
#define AGENT_MAX_SPEED 1.0f
#define AGENT_MAX_FORCE 1.0f
#define AGENT_RADIUS 0.5f
#define AGENT_QUERY_RADIUS 1.0f

Util::Vector CogAlgorithm::compute_velocity_force()
{
	// Compute the velocity force
	Util::Vector velocity_force = Util::Vector(0, 0, 0);
	return velocity_force;
}

Util::Vector CogAlgorithm::compute_agent_collision_force()
{
	// Compute the agent collision force
	Util::Vector agent_collision_force = Util::Vector(0, 0, 0);
	return agent_collision_force;
}

Util::Vector CogAlgorithm::compute_obstacle_collision_force()
{
	// Compute the obstacle collision force
	Util::Vector obstacle_collision_force = Util::Vector(0.1, 0, 0);
	return obstacle_collision_force;
}

void CogAlgorithm::check_neighbors()
{
	// Check the neighbors
	_neighbors.clear();
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
			_position.x-(this->_radius + AGENT_QUERY_RADIUS),
			_position.x+(this->_radius + AGENT_QUERY_RADIUS),
			_position.z-(this->_radius + AGENT_QUERY_RADIUS),
			_position.z+(this->_radius + AGENT_QUERY_RADIUS),
			dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
	// TODO: Separate the neighbors into agents and obstacles, now all are considered agents

}

void CogAlgorithm::apply_rigid_body_force(const Util::Vector & force, float dt)
{
	// Update the rigid body
	auto clipped_force = Util::clamp(force, AGENT_MAX_FORCE);
	auto acceleration = clipped_force / AGENT_MASS;

	_velocity += acceleration * dt;
	_velocity = Util::clamp(_velocity, AGENT_MAX_SPEED);

	auto new_position = _position + _velocity * dt;
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(new_position.x - _radius, new_position.x + _radius, 0.0f, 0.0f, new_position.z - _radius, new_position.z + _radius);
	gSpatialDatabase->updateObject(this, oldBounds, newBounds);

	_position = new_position;
}

