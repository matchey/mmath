

#ifndef MMATH_COMMON_ANGLES_IMPL_HPP_
#define MMATH_COMMON_ANGLES_IMPL_HPP_

#include <cmath>

namespace mmath
{
	inline float
		normAngle (float alpha)
		{
			return (alpha >= 0  ? 
					fmodf (alpha + static_cast<float>(M_PI), 
						2.0f * static_cast<float>(M_PI)) 
					- static_cast<float>(M_PI) 
					: 
					-(fmodf (static_cast<float>(M_PI) - alpha, 
							2.0f * static_cast<float>(M_PI)) 
						- static_cast<float>(M_PI)));
		}

	inline float
		acuteAngle (float alpha)
		{
			return (alpha >= 0  ?
					fmodf (alpha + static_cast<float>(1.570796f),
						static_cast<float>(M_PI))
					- static_cast<float>(1.570796f)
					:
					-(fmodf (static_cast<float>(1.570796f) - alpha,
							static_cast<float>(M_PI))
						- static_cast<float>(1.570796f)));
		}


	inline float 
		rad2deg (float alpha)
		{
			return (alpha * 57.29578f);
		}

	inline float 
		deg2rad (float alpha)
		{
			return (alpha * 0.017453293f);
		}

	inline double 
		rad2deg (double alpha)
		{
			return (alpha * 57.29578);
		}

	inline double 
		deg2rad (double alpha)
		{
			return (alpha * 0.017453293);
		}
}

#endif  // MMATH_COMMON_ANGLES_IMPL_HPP_

