#pragma once

#include <cmath>
#include <cstdlib>

namespace CogUtils {

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

}
