
#ifndef MMATH_COMMON_LOGISTIC_H_
#define MMATH_COMMON_LOGISTIC_H_

namespace mmath
{
	class LogisticFunction{
		public:
		LogisticFunction();

		private:
		double rate;
	};
	double logistic(const double&);
}

#include <mmath/common/impl/logistic.hpp>

#endif  // MMATH_COMMON_LOGISTIC_H_

