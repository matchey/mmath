
#include <geometry_msgs/Pose.h>
#include "mmath/differential.h"
#include "mmath/binarion.h"

Differential::Differential()
	: pre_x(0.0), pre_y(0.0), pre_z(0.0), length(0.0), isAngle(false), isBegin(true)
{
	initialize();
}

Differential::Differential(const double& x)
	: pre_x(x), length(0.0), isAngle(false), isBegin(false)
{
	initialize();
}

Differential::Differential(const double& x, const bool flag)
	: pre_x(x), length(0.0), isAngle(flag), isBegin(false)
{
	initialize();
}

Differential::Differential(const double& x, const double& y)
	: pre_x(x), pre_y(y), length(0.0), isAngle(false), isBegin(false)
{
	initialize();
}

Differential::Differential(const double& x, const double& y, const double& z)
	: pre_x(x), pre_y(y), pre_z(z), length(0.0), isAngle(false), isBegin(false)
{
	initialize();
}

Differential::Differential(const pcl::PointXYZ& p)
	: pre_x(p.x), pre_y(p.y), pre_z(p.z), length(0.0), isAngle(false), isBegin(false)
{
	initialize();
}

Eigen::Vector3d Differential::get()
{
	return v;
}

double Differential::set(const double& x)
{
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(dt < 1e-6 || isBegin){
		length = 0.0;
		isBegin = false;
	}else{
		length = isAngle ? Binarion::deviation(pre_x, x) / dt : (x - pre_x) / dt;
	}

	pre_x = x;

	return length;
}

Eigen::Vector2d Differential::set(const double& x, const double& y)
{
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(dt < 1e-6 || isBegin){
		length = 0.0;
		isBegin = false;
	}else{
		length = sqrt(pow(x - pre_x, 2) + pow(y - pre_y, 2)) / dt;
		v(0) = (x - pre_x) / dt;
		v(1) = (y - pre_y) / dt;
	}

	pre_x = x;
	pre_y = y;

	return v.block<2, 1>(0, 0);
}

Eigen::Vector3d Differential::set(const double& x, const double& y, const double& z)
{
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(dt < 1e-6 || isBegin){
		length =  0.0;
		isBegin = false;
	}else{
		length = sqrt(pow(x - pre_x, 2) + pow(y - pre_y, 2) + pow(z - pre_z, 2)) / dt;
		v(0) = (x - pre_x) / dt;
		v(1) = (y - pre_y) / dt;
		v(2) = (z - pre_z) / dt;
	}

	pre_x = x;
	pre_y = y;
	pre_z = z;

	return v;
}

// double Differential::get(const pcl::PointXYZ& p)
// {
// 	return get(p.x, p.y, p.z);
// }

template<class T_p>
Eigen::Vector3d Differential::set(const T_p& p)
{
	return set(p.x, p.y, p.z);
}
template Eigen::Vector3d Differential::set<geometry_msgs::Point>(const geometry_msgs::Point& p);

double Differential::norm() const
{
	return length;
}

void Differential::setFlagAngle()
{
	isAngle = true;
}

void Differential::setFlagAngle(const bool flag)
{
	isAngle = flag;
}

////////// private ///////////
void Differential::initialize()
{
	last_time = ros::Time::now();
	current_time = ros::Time::now();
}

