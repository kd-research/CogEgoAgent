#include <cmath>
#include <vector>

#include "CogAlgorithm.h"
#include "imgui.h"
#include "utils.h"

extern SteerLib::EngineInterface *gEngine;
extern SteerLib::SpatialDataBaseInterface *gSpatialDatabase;

#define EPSILON 0.0001f

Util::Vector CogAlgorithm::compute_velocity_force()
{
    if (_velocity.length() <= EPSILON)
    {
        _velocity = Util::Vector(CogUtils::NextGaussian(), 0, CogUtils::NextGaussian());
    }

    std::vector<Util::Vector> visions(parameters.visionResolution);
    for (int i = 0; i < parameters.visionResolution; i++)
    {
        float angle = parameters.visionPhi * 2 * i / parameters.visionResolution - parameters.visionPhi;
        visions[i] = Util::normalize(Util::rotateInXZPlane(_velocity, angle));
    }

    std::vector<float> vision_intersections(parameters.visionResolution);
    for (int i = 0; i < parameters.visionResolution; i++)
    {
        SteerLib::SpatialDatabaseItemPtr hititem;
        Util::Ray ray;
        ray.initWithLengthInterval(_position, visions[i] * parameters.visionRange);
        float hit_t = 0;
        if (gSpatialDatabase->trace(ray, hit_t, hititem, this, false))
        {
            vision_intersections[i] = hit_t;
        }
        else
        {
            vision_intersections[i] = parameters.visionRange;
        }
    }

    std::vector<float> utility_values(parameters.visionResolution);
    float minUtility = std::numeric_limits<float>::max();
    int bestDirectionIndex = 0;
    Util::Vector bestDirection;
    Util::Vector goaldirection = normalize(currentGoal().targetLocation - _position);

    for (int i = 0; i < parameters.visionResolution; i++)
    {
        float cos_angle = Util::dot(visions[i], goaldirection);
        float thisUtility = pow(parameters.visionRange, 2) + pow(vision_intersections[i], 2) -
                            2.0 * cos_angle * parameters.visionRange * vision_intersections[i];
        utility_values[i] = thisUtility;
        if (thisUtility < minUtility)
        {
            minUtility = thisUtility;
            bestDirection = visions[i];
            bestDirectionIndex = i;
        }
    }

    Util::Vector expectedVelocity = bestDirection * vision_intersections[bestDirectionIndex] / parameters.visionTau;
    expectedVelocity = Util::clamp(expectedVelocity, parameters.maxSpeed);

    return (expectedVelocity - _velocity) / parameters.visionTau;
}

Util::Vector CogAlgorithm::compute_agent_collision_force()
{
    // Compute the agent collision force
    Util::Vector agent_collision_force = Util::Vector(0, 0, 0);
    for (auto &neighbor : _collisionAgents)
    {
        Util::Vector relative_direction = (_position - neighbor->position());
        if ((relative_direction).length() > EPSILON)
        {
            relative_direction = Util::normalize(relative_direction);
        }
        else
        {
            relative_direction =
                Util::rotateInXZPlane(Util::Vector(1, 0, 0), (static_cast<float>(rand()) / RAND_MAX) * 2 * M_PI);
        }
        bool overlaps = neighbor->overlaps(_position, _radius);
        if (!overlaps)
        {
            continue;
        }
        agent_collision_force += relative_direction * parameters.k / parameters.mass;
    }
    return agent_collision_force;
}

Util::Vector CogAlgorithm::compute_obstacle_collision_force()
{
    Util::Vector obstacle_collision_force = Util::Vector(0, 0, 0);
    Util::Vector intersect_norm = Util::Vector(0, 0, 0);

    for (auto &neighbor : _collisionObstacles)
    {
        if (!neighbor->overlaps(_position, _radius))
            continue;

        do
        {
            SteerLib::BoxObstacle *box = dynamic_cast<SteerLib::BoxObstacle *>(neighbor);
            if (box)
            {
                CogUtils::CalculateBoxPointNorm(box->getBounds(), _position, intersect_norm);
                break;
            }

            SteerLib::OrientedBoxObstacle *oriented_box = dynamic_cast<SteerLib::OrientedBoxObstacle *>(neighbor);
            if (oriented_box)
            {
                oriented_box->getDistance(_position, intersect_norm);
                intersect_norm = Util::normalize(intersect_norm);
                break;
            }

            throw std::runtime_error("Unknown obstacle type");
        } while (0);

        obstacle_collision_force += intersect_norm * parameters.k / parameters.mass;
    }
    return obstacle_collision_force;
}

void CogAlgorithm::correct_wall_intrusion(Util::Point &position, Util::Vector &velocity)
{
    if (_collisionObstacles.empty())
        return;

    Util::Vector intersect_sum = Util::Vector(0, 0, 0);
    for (auto &neighbor : _collisionObstacles)
    {
        Util::Vector intersect_norm = Util::Vector(0, 0, 0);
        do
        {
            SteerLib::BoxObstacle *box = dynamic_cast<SteerLib::BoxObstacle *>(neighbor);
            if (box)
            {
                CogUtils::CalculateBoxPointNorm(box->getBounds(), position, intersect_norm);
                break;
            }

            SteerLib::OrientedBoxObstacle *oriented_box = dynamic_cast<SteerLib::OrientedBoxObstacle *>(neighbor);
            if (oriented_box)
            {
                oriented_box->getDistance(position, intersect_norm);
                intersect_norm = Util::normalize(intersect_norm);
                break;
            }

            throw std::runtime_error("Unknown obstacle type");
        } while (0);

        float peneration = neighbor->computePenetration(position, _radius);
        intersect_sum = intersect_sum + intersect_norm;

        float dot = Util::dot(velocity, intersect_sum);
        if (dot > 0)
            continue;

        position = position + intersect_norm * peneration;
        velocity = velocity - dot * intersect_sum;
    }

    if (intersect_sum.length() < EPSILON)
        return;

    float dot = Util::dot(velocity, intersect_sum);
    if (dot < 0)
        velocity = velocity - (0.1 + dot) * intersect_sum;

    if (velocity.length() < 0.1)
        velocity = 0.3 * intersect_sum;
}

void CogAlgorithm::apply_rigid_body_force(const Util::Vector &force, float dt)
{
    // Update the rigid body
    auto clipped_force = Util::clamp(force, parameters.maxForce);
    // auto acceleration = clipped_force / parameters.mass;  // Acceleration type force ignores agent mass
    auto acceleration = clipped_force;

    _velocity += acceleration * dt;
    _velocity = Util::clamp(_velocity, parameters.maxSpeed);
    _forward = Util::normalize(_velocity);

    auto new_position = _position + _velocity * dt;
    correct_wall_intrusion(new_position, _velocity);

    Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius,
                                   _position.z + _radius);
    Util::AxisAlignedBox newBounds(new_position.x - _radius, new_position.x + _radius, 0.0f, 0.0f,
                                   new_position.z - _radius, new_position.z + _radius);
    gSpatialDatabase->updateObject(this, oldBounds, newBounds);

    _position = new_position;
}

void CogAlgorithm::draw()
{
    if (_config->showAgentSelectedOnly && !gEngine->isAgentSelected(this))
    {
        return;
    }

    if (_config->showAgentInfo)
    {
        ImGui::BeginGroup();
        if (gEngine->isAgentSelected(this))
        {
            ImGui::Text("Agent: %ld (selected)", _id);
        }
        else
        {
            ImGui::Text("Agent: %ld", _id);
        }

        Util::Vector velocityForce = compute_velocity_force();
        Util::Vector agentCollisionForce = compute_agent_collision_force();
        Util::Vector obstacleCollisionForce = compute_obstacle_collision_force();
        ImGui::Text("  Velocity force: (%f, %f, %f)", velocityForce.x, velocityForce.y, velocityForce.z);
        ImGui::Text("  Agent neighbor size: %ld", _collisionAgents.size());
        ImGui::Text("  Agent collision force: (%f, %f, %f)", agentCollisionForce.x, agentCollisionForce.y,
                    agentCollisionForce.z);
        ImGui::Text("  Obstacle neighbor size: %ld", _collisionObstacles.size());
        ImGui::Text("  Obstacle collision force: (%f, %f, %f)", obstacleCollisionForce.x, obstacleCollisionForce.y,
                    obstacleCollisionForce.z);
        ImGui::EndGroup();
    }
}
