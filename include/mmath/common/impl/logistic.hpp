
#ifndef MMATH_COMMON_LOGISTIC_HPP_
#define MMATH_COMMON_LOGISTIC_HPP_

// #include <cmath>

namespace mmath
{
	LogisticFunction::LogisticFunction()
	{
	}

	double logistic(const double& t) // 仮
	{
		const double K = 1;
		const double N0 = 0.19;
		const double r = 1.3;

		return K / ( 1 + (K/N0 - 1) * exp(-r*t) );
	}
}

#endif  // MMATH_COMMON_LOGISTIC_HPP_

