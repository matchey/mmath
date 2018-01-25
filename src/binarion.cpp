
#include <cmath>
#include <ostream>
#include "mmath/binarion.h"

Binarion::Binarion()
	: theta(0.0), x(1.0), y(0.0)
{
}

Binarion::Binarion(const double &yaw, const std::string str)
	: theta(yaw), x(cos(yaw)), y(sin(yaw))
{
	if(str == "deg" || str == "degree"){
		theta = yaw * M_PI / 180;
		x = cos(theta);
		y = sin(theta);
	}
}

Binarion::Binarion(const Binarion &b)
	: theta(b.theta), x(b.x), y(b.y)
{
}

Binarion::Binarion(const double& bx, const double& by)
	: theta(atan2(by, bx)), x(bx), y(by)
{
}

Binarion Binarion::fromYaw(const double &yaw, const std::string str)
{
	Binarion b(yaw, str);

	return b;
}

double Binarion::toYaw(const Binarion& bina, const std::string str)
{
	return bina.getYaw(str);
}

double Binarion::getYaw(const std::string str) const
{
	if(str == "deg" || str == "degree"){
		return theta * 180 / M_PI;
	}else{
		return theta;
	}
}

double Binarion::deviation(const double& src, const double& tgt, const std::string str)
{
	return fromYaw(src, str).deviation(tgt, str);
}

double Binarion::deviation(const double& tgt, const std::string str) const
{
	return (fromYaw(tgt, str) - *this).getYaw(str);
}

double Binarion::deviation(const Binarion& tgt, const std::string str) const
{
	return (tgt - *this).getYaw(str);
}

Binarion Binarion::slerp(const Binarion& bina, double t) const
{
	Binarion rtn(*this);
	double th = this->deviation(bina);

	if(t > 1.0) t = 1.0;
	if(t < 0.0) t = 0.0;

	rtn += t * fromYaw(th);

	// return (sin((1-t)*th)*(*this) + sin(t*th)*bina) / sin(th);
	return rtn;
}

// template<class F_b, class... T_b>
// Binarion Binarion::mean(F_b first, T_b... args) const
// {
// 	int i = 0;
// 	Binarion binarion(first);
//
// 	for(Binarion bina : std::initializer_list<Binarion>{args...}){
// 		binarion = binarion.slerp(bina, 1.0/(++i));
// 	}
//
// 	return binarion;
// }
// template Binarion Binarion::mean<Binarion>(Binarion) const;
// template Binarion Binarion::mean<Binarion, Binarion>(Binarion, Binarion) const;
// template Binarion Binarion::mean<Binarion, Binarion, Binarion>(Binarion, Binarion, Binarion) const;

Binarion Binarion::operator + () const
{
	return *this;
}

Binarion Binarion::operator - () const
{
	Binarion bina;

	bina.x =  cos(this->theta);
	bina.y = -sin(this->theta);
	bina.theta = atan2(bina.y, bina.x);

	return bina;
}

Binarion Binarion::operator + (const Binarion& rhs) const
{
	Binarion bina;

	bina.x = cos(rhs.theta) * this->x - sin(rhs.theta) * this->y;
	bina.y = sin(rhs.theta) * this->x + cos(rhs.theta) * this->y;
	bina.theta = atan2(bina.y, bina.x);

	return bina;
}

Binarion Binarion::operator - (const Binarion& rhs) const
{
	return *this + (-rhs);
}

Binarion Binarion::operator * (const int& rhs) const
{
	return Binarion::fromYaw(theta * rhs);
}

Binarion Binarion::operator * (const double& rhs) const
{
	return Binarion::fromYaw(theta * rhs);
}

Binarion Binarion::operator / (const double& rhs) const
{
	return Binarion::fromYaw(theta / rhs);
}

Binarion& Binarion::operator += (const Binarion& rhs)
{
	*this = *this + rhs;
	
	return *this;
}

Binarion& Binarion::operator -= (const Binarion& rhs)
{
	*this = *this - rhs;
	
	return *this;
}

bool Binarion::operator == (const Binarion& rhs) const
{
	return x == rhs.x && y == rhs.y;
}

bool Binarion::operator != (const Binarion& rhs) const
{
	return !(*this == rhs);
}

double Binarion::operator () (const int& n) const
{
	if(n == 0){
		return this->x;
	}else if(n == 1){
		return this->y;
	}else{
		return 0.0;
	}
}

Binarion operator * (const int& lhs, const Binarion &rhs)
{
	return rhs * lhs;
}

Binarion operator * (const double& lhs, const Binarion &rhs)
{
	return rhs * lhs;
}

std::ostream& operator << (std::ostream &os, const Binarion &rhs)
{
	os << "(" << rhs.x << ", " << rhs.y << ")";

	return os;
}

