#include <array>

#include "CogAlgorithm.h"
#include "imgui.h"
#include "utils.h"

extern SteerLib::EngineInterface *gEngine;
extern SteerLib::SpatialDataBaseInterface *gSpatialDatabase;

#define EPSILON 0.0001f
#define AGENT_MASS 80.0f
#define AGENT_MAX_SPEED 1.5f
#define AGENT_MAX_FORCE 9999.0f
#define AGENT_RADIUS 0.5f
#define AGENT_QUERY_RADIUS 1.0f
#define COG_VISION_PHI 1.30899f // 75 degrees
#define COG_VISION_TAU 0.5f
#define COG_VISION_RESOLUTION 50
#define COG_VISION_RANGE 10.0f
#define COG_AGENT_K 5000.0f

Util::Vector CogAlgorithm::compute_velocity_force()
{
    if (_velocity.length() <= EPSILON)
    {
        _velocity = Util::Vector(CogUtils::NextGaussian(), 0, CogUtils::NextGaussian());
    }

    std::array<Util::Vector, COG_VISION_RESOLUTION> visions;
    for (int i = 0; i < COG_VISION_RESOLUTION; i++)
    {
        float angle = COG_VISION_PHI * 2 * i / COG_VISION_RESOLUTION - COG_VISION_PHI;
        visions[i] = Util::normalize(Util::rotateInXZPlane(_velocity, angle));
    }

    std::array<float, COG_VISION_RESOLUTION> vision_intersections;
    for (int i = 0; i < COG_VISION_RESOLUTION; i++)
    {
        SteerLib::SpatialDatabaseItemPtr hititem;
        Util::Ray ray;
        ray.initWithLengthInterval(_position, visions[i] * COG_VISION_RANGE);
        float hit_t = 0;
        if (gSpatialDatabase->trace(ray, hit_t, hititem, this, false))
        {
            vision_intersections[i] = hit_t;
        }
        else
        {
            vision_intersections[i] = COG_VISION_RANGE;
        }
    }

    std::array<float, COG_VISION_RESOLUTION> utility_values;
    float minUtility = std::numeric_limits<float>::max();
    int bestDirectionIndex = 0;
    Util::Vector bestDirection;
    Util::Vector goaldirection = normalize(currentGoal().targetLocation - _position);

    for (int i = 0; i < COG_VISION_RESOLUTION; i++)
    {
        float cos_angle = Util::dot(visions[i], goaldirection);
        float thisUtility = pow(COG_VISION_RANGE, 2) + pow(vision_intersections[i], 2) -
                            2.0 * cos_angle * COG_VISION_RANGE * vision_intersections[i];
        utility_values[i] = thisUtility;
        if (thisUtility < minUtility)
        {
            minUtility = thisUtility;
            bestDirection = visions[i];
            bestDirectionIndex = i;
        }
    }

    Util::Vector expectedVelocity = bestDirection * vision_intersections[bestDirectionIndex] / COG_VISION_TAU;
    expectedVelocity = Util::clamp(expectedVelocity, AGENT_MAX_SPEED);

    return (expectedVelocity - _velocity) / COG_VISION_TAU;
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
        agent_collision_force += relative_direction * COG_AGENT_K / AGENT_MASS;
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

        obstacle_collision_force += intersect_norm * COG_AGENT_K / AGENT_MASS;
    }
    return obstacle_collision_force;
}

void CogAlgorithm::apply_rigid_body_force(const Util::Vector &force, float dt)
{
    // Update the rigid body
    auto clipped_force = Util::clamp(force, AGENT_MAX_FORCE);
    // auto acceleration = clipped_force / AGENT_MASS;  // Acceleration type force ignores agent mass
    auto acceleration = clipped_force;

    _velocity += acceleration * dt;
    _velocity = Util::clamp(_velocity, AGENT_MAX_SPEED);
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
