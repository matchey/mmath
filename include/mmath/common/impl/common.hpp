

#ifndef MMATH_COMMON_IMPL_HPP_
#define MMATH_COMMON_IMPL_HPP_

#include "mmath/common.h"

inline double
max (const double& a, const double& b)
{
	return a > b ? a : b;
}

inline double
min (const double& a, const double& b)
{
	return a < b ? a : b;
}

#endif  // MMATH_COMMON_IMPL_HPP_

