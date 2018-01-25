
#ifndef PCA_H
#define PCA_H

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// template<class T>
struct Points2D
{
	struct point2d{
		double x;
		double y;
	};
	std::vector<point2d> points;
	// Points2D& operator -> () const noexcept;
};

class PrincipalComponentAnalysis
{
	double curv;
	Eigen::Vector3d values;
	Eigen::Matrix3d vectors;
	Eigen::Vector3d center;

	double angle;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_saved;
	int cnt = 0;

	void calcVcov(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Matrix3d&);
	void setPoints3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	void setVector(const Eigen::Vector2d&, const Eigen::Vector2d&);

	public:
	PrincipalComponentAnalysis();
	void setPoints2d(const pcl::PointXYZ&, const unsigned int&);
	void setPoints2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	template<class T_p>
	void setPoints(const T_p&);
	// void setPoints(const nav_msgs::Odometry::ConstPtr&);
	// void setPoints(const geometry_msgs::Point&);
	// void setPoints2D(const geometry_msgs::Point&);
	double curvature();
	double value(const int&); // ordinal
	Eigen::Vector3d vector(const int&); // ordinal
	double direction() const;
};

#endif

