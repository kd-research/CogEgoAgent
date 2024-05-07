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
        Util::Vector relative_direction = Util::normalize(_position - neighbor->position());
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
                break;
            }
            std::cerr << "Unknown obstacle type" << std::endl;
            exit(1);
        } while (0);

        obstacle_collision_force += intersect_norm * parameters.k / parameters.mass;
    }
    return obstacle_collision_force;
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
    Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius,
                                   _position.z + _radius);
    Util::AxisAlignedBox newBounds(new_position.x - _radius, new_position.x + _radius, 0.0f, 0.0f,
                                   new_position.z - _radius, new_position.z + _radius);
    gSpatialDatabase->updateObject(this, oldBounds, newBounds);

    _position = new_position;
}

void CogAlgorithm::draw()
{
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
        ImGui::Text("Agent neighbor size: %ld", _collisionAgents.size());
        ImGui::Text("Obstacle neighbor size: %ld", _collisionObstacles.size());
        ImGui::EndGroup();
    }
}
