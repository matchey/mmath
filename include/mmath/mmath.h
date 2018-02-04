
#ifndef MATCHEY_MATH_H
#define MATCHEY_MATH_H

namespace mmath
{
	template<class T1, class T2>
	void pointSubstitution(T1& p1, const T2& p2)
	{
		p1.x = p2.x;
		p1.y = p2.y;
		p1.z = p2.z;
	}

	template<class T1, class T2>
	void quatSubstitution(T1& q1, const T2& q2)
	{
		q1.x = q2.x;
		q1.y = q2.y;
		q1.z = q2.z;
		q1.w = q2.w;
	}
}

#endif

