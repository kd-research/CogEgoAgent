#pragma once

#include <cmath>
#include <cstdlib>

namespace CogUtils
{

float NextGaussian()
{
    float v1, v2, s;
    do
    {
        v1 = 2.0f * rand() / RAND_MAX - 1;
        v2 = 2.0f * rand() / RAND_MAX - 1;
        s = v1 * v1 + v2 * v2;
    } while (s >= 1 || s == 0);
    s = sqrt((-2.0f * log(s)) / s);

    return v1 * s;
}

bool CalculateBoxPointNorm(const Util::AxisAlignedBox &box, const Util::Point &point, Util::Vector &normal)
{
    if (point.x > box.xmin && point.x < box.xmax && point.z > box.zmin && point.z < box.zmax)
    {
        auto center = Util::Point((box.xmin + box.xmax) / 2, 0, (box.zmin + box.zmax) / 2);
        auto diff = point - center;
        normal = Util::Vector(diff.x, 0, diff.z);
        normal = Util::normalize(normal);
        return true;
    }
    else
    {
        if (point.x < box.xmin)
        {
            normal += Util::Vector(-1, 0, 0);
        }
        if (point.x > box.xmax)
        {
            normal += Util::Vector(1, 0, 0);
        }
        if (point.z < box.zmin)
        {
            normal += Util::Vector(0, 0, -1);
        }
        if (point.z > box.zmax)
        {
            normal += Util::Vector(0, 0, 1);
        }
        normal = Util::normalize(normal);
        return false;
    }
}

} // namespace CogUtils
