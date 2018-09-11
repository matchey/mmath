
#ifndef BINARION_H
#define BINARION_H

class Binarion
{
	double theta;
	double x;
	double y;

	Binarion(const double&, const double&); //x, y

	public:
	Binarion();
	Binarion(const double&, const std::string str = "radian"); //theta
	Binarion(const Binarion&); // copy
	static Binarion fromYaw(const double&, const std::string str = "radian"); //theta
	static double toYaw(const Binarion&, const std::string str = "radian");
	double getYaw(const std::string str = "radian") const;
	static double deviation(const double&, const double&, const std::string str = "radian");
	double deviation(const double&, const std::string str = "radian") const; //theta2 - this.theta
	double deviation(const Binarion&, const std::string str = "radian") const; //theta2 - this.theta
	Binarion slerp(const Binarion&, double) const; //return arg@t=1 this@t=0
	template<class... T_b>
	Binarion mean(Binarion, T_b...) const;
	template<class... T_th>
	static double mean(double first, T_th...);
	template<class... T_th>
	static double mean(std::string, double first, T_th...);

	inline Binarion operator + () const;
	Binarion operator - () const;
	Binarion operator + (const Binarion&) const;
	Binarion operator - (const Binarion&) const;
	Binarion operator * (const int&) const;
	Binarion operator * (const double&) const;
	Binarion operator / (const double&) const;
	Binarion& operator += (const Binarion&);
	Binarion& operator -= (const Binarion&);
	bool operator == (const Binarion&) const;
	inline bool operator != (const Binarion&) const;
	double operator () (const int&) const;
	friend Binarion operator * (const int&, const Binarion&);
	friend Binarion operator * (const double&, const Binarion&);
	friend std::ostream& operator << (std::ostream&, const Binarion&);
};

#include "mmath/binarion/mean.h" // impl for using template args

#endif

