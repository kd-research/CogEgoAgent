#pragma once

#define AGENT_MASS 80.0f
#define AGENT_MAX_SPEED 1.5f
#define AGENT_MAX_FORCE 9999.0f
#define AGENT_QUERY_RADIUS 1.0f
#define AGENT_DRAG 1.0f
#define AGENT_WALL_COLLISION_COEFFICIENT 0.1f
#define COG_VISION_PHI 1.30899f // 75 degrees
#define COG_VISION_TAU 0.5f
#define COG_VISION_RESOLUTION 50
#define COG_VISION_RANGE 10.0f
#define COG_AGENT_K 5000.0f

struct CogAgentParameters
{
    float mass;
    float maxSpeed;
    float maxForce;
    float queryRadius;
    float visionPhi;
    float visionTau;
    int visionResolution;
    float visionRange;
    float k;
    float drag;
    float wallcol_mu;

    CogAgentParameters()
    {
        mass = AGENT_MASS;
        maxSpeed = AGENT_MAX_SPEED;
        maxForce = AGENT_MAX_FORCE;
        queryRadius = AGENT_QUERY_RADIUS;
        visionPhi = COG_VISION_PHI;
        visionTau = COG_VISION_TAU;
        visionResolution = COG_VISION_RESOLUTION;
        visionRange = COG_VISION_RANGE;
        k = COG_AGENT_K;
        drag = AGENT_DRAG;
        wallcol_mu = AGENT_WALL_COLLISION_COEFFICIENT;
    }
};
