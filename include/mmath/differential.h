

#ifndef DIFFERENTIAL_H
#define DIFFERENTIAL_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

class Differential
{
	Eigen::Vector3d v;
	double pre_x;
	double pre_y;
	double pre_z;
	double length;
	bool isAngle;
	bool isBegin;
	ros::Time current_time, last_time;

	void initialize();

	public:
	Differential();
	Differential(const double&);
	Differential(const double&, const bool);
	Differential(const double&, const double&);
	Differential(const double&, const double&, const double&);
	Differential(const pcl::PointXYZ&);
	Eigen::Vector3d get();
	double set(const double&); // x --> dx/dt
	Eigen::Vector2d set(const double&, const double&); // x, y --> vx, vy
	Eigen::Vector3d set(const double&, const double&, const double&); //x, y, z --> vx, vy, vz
	// Eigen::Vector3d get(const pcl::PointXYZ&); //x, y, z --> vx, vy, vz
	template<class T_p>
	Eigen::Vector3d set(const T_p&); //x, y, z --> v
	double norm() const;
	void setFlagAngle();
	void setFlagAngle(const bool);
};

#endif

