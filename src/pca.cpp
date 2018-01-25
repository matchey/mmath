
#include <Eigen/Dense> // for EigenSolver
#include "mmath/pca.h"

// Points2D& Points2D::operator -> () const noexcept
// {
// 	return *this;
// }

PrincipalComponentAnalysis::PrincipalComponentAnalysis()
	: curv(1.0), values(Eigen::Vector3d::Ones()),
	  vectors(Eigen::Matrix3d::Identity()), center(Eigen::Vector3d::Zero()),
	  pc_saved(new pcl::PointCloud<pcl::PointXYZ>), cnt(0)
{
}

double PrincipalComponentAnalysis::curvature()
{
	return curv;
}

double PrincipalComponentAnalysis::value(const int& ordinal)
{
	return values(ordinal);
}

Eigen::Vector3d PrincipalComponentAnalysis::vector(const int& ordinal)
{
	return vectors.col(ordinal);
}

double PrincipalComponentAnalysis::direction() const
{
	return angle;
}

void PrincipalComponentAnalysis::setPoints2d(const pcl::PointXYZ& p, const unsigned int& size)
{
	if(pc_saved->points.size() < size){
		pc_saved->points.push_back(p);
	}else{
		angle = atan2(p.y - pc_saved->points[cnt].y, p.x - pc_saved->points[cnt].x);
		pc_saved->points[cnt] = p;
		setPoints(pc_saved);
	}
	cnt = (cnt+1)%size;
}

void PrincipalComponentAnalysis::setPoints2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
	setPoints(pc);
}

template<class T_p>
void PrincipalComponentAnalysis::setPoints(const T_p& points)
{
	int n = points->points.size();
	if(!n){
		std::cerr << "POINTS SIZE IS ZERO!!!!1!! in PCA" << std::endl;
		curv= 1.0;
		values = Eigen::Vector3d::Ones();
		vectors = Eigen::Matrix3d::Identity();
		center = Eigen::Vector3d::Zero();
		return;
	}
	double cov  = 0.0;
	double varx = 0.0; double vary = 0.0;
	double x_sum = 0.0; double y_sum = 0.0;
	double x_ave = 0.0; double y_ave = 0.0;
	double prodsum_xy = 0.0; double prodsum_xx = 0.0; double prodsum_yy = 0.0;
	Eigen::Matrix2d vcov;

	for(auto itr = points->points.begin(); itr != points->points.end(); ++itr){
		x_sum += itr->x;
		y_sum += itr->y;
		prodsum_xx += itr->x * itr->x;
		prodsum_yy += itr->y * itr->y;
		prodsum_xy += itr->x * itr->y;
	}
	x_ave = x_sum / n;
	y_ave = y_sum / n;

	center << x_ave, y_ave , 0.0;

	cov  = prodsum_xy / n - x_ave * y_ave;
	varx = prodsum_xx / n - x_ave * x_ave;
	vary = prodsum_yy / n - y_ave * y_ave;

	vcov << varx, cov,
		 	cov, vary;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	if(!es.info()){ // == "Success"
		Eigen::Vector2d eigenvalues  = es.eigenvalues().real();
		Eigen::Matrix2d eigenvectors = es.eigenvectors().real();

		if(eigenvalues(0) < eigenvalues(1)){
			values(0) = eigenvalues(1); // first
			values(1) = eigenvalues(0); // second
			setVector(eigenvectors.col(1), eigenvectors.col(0));
		}else{
			values(0) = eigenvalues(0); // first
			values(1) = eigenvalues(1); // second
			setVector(eigenvectors.col(0), eigenvectors.col(1));
		}

		if(values(0) + values(1) < 1e-6){
			curv= 1.0;
		}else{
			curv= 2.0 * values(1) / ( values(0) + values(1) );
		}
	}else{
		curv = 1.0;
		std::cerr << "Eigen solver error info : " << es.info() << std::endl;
	}
}
// template void PrincipalComponentAnalysis::setPoints<Points2D>(const Points2D&);
template void PrincipalComponentAnalysis::setPoints<pcl::PointCloud<pcl::PointXYZ>::Ptr>(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);

void PrincipalComponentAnalysis::calcVcov(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, Eigen::Matrix3d& vcov)
{
    int n = pc->points.size();
    if(!n){
		std::cerr << "!!!!!!! POINTS SIZE IS ZERO in PCA !!!!1!!" << std::endl;
		curv= 1.0;
		values = Eigen::Vector3d::Ones();
		vectors = Eigen::Matrix3d::Identity();
		center = Eigen::Vector3d::Zero();
        return;
    }

    double s_xx, s_xy, s_xz,
		   s_yy, s_yz, s_zz,
           x_sum, y_sum, z_sum,
		   x_ave, y_ave, z_ave,
		   xx_sum, yy_sum, zz_sum, xy_sum, xz_sum, yz_sum;

    x_sum = y_sum = z_sum
		  = x_ave = y_ave = z_ave
          = xx_sum = yy_sum = zz_sum = xy_sum = xz_sum = yz_sum = 0.0;

    for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
        x_sum += it->x; y_sum += it->y; z_sum += it->z;
        xx_sum += it->x * it->x; yy_sum += it->y * it->y; zz_sum += it->z * it->z;
        xy_sum += it->x * it->y; xz_sum += it->x * it->z; yz_sum += it->y * it->z;
    }

    x_ave = x_sum / n;
    y_ave = y_sum / n;
    z_ave = z_sum / n;

	center << x_ave, y_ave, z_ave;

    s_xx = xx_sum / n - x_ave * x_ave;
    s_yy = yy_sum / n - y_ave * y_ave;
    s_zz = zz_sum / n - z_ave * z_ave;
    s_xy = xy_sum / n - x_ave * y_ave;
    s_xz = xz_sum / n - x_ave * z_ave;
    s_yz = yz_sum / n - y_ave * z_ave;

    vcov << s_xx, s_xy, s_xz,
            s_xy, s_yy, s_yz,
            s_xz, s_yz, s_zz;
}

void PrincipalComponentAnalysis::setPoints3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
    Eigen::Matrix3d vcov;

	calcVcov(pc, vcov);

    Eigen::EigenSolver<Eigen::Matrix3d> es(vcov);
    Eigen::Vector3d eigenvalues = es.eigenvalues().real();
    Eigen::Matrix3d eigenvectors = es.eigenvectors().real();

    int min = 0;
	int mid = 0;
	int max = 0;

    for(int i = 1; i < 3; ++i){
        min = eigenvalues(i) < eigenvalues(min) ? i : min;
        max = eigenvalues(i) > eigenvalues(max) ? i : max;
    }
	mid = (3 - min - max);

	values(0) = eigenvalues(max);
	values(1) = eigenvalues(mid);
	values(2) = eigenvalues(min);

	vectors.col(0) = eigenvectors.col(max);
	vectors.col(1) = eigenvectors.col(mid);
	vectors.col(2) = eigenvectors.col(min);

    double lambda_sum = values(0) + values(1) + values(2);

    if(lambda_sum < 1e-6){
		curv= 1.0;
	}else{
		curv= 3.0 * values(2) / lambda_sum;
	}
}

void PrincipalComponentAnalysis::setVector(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
	vectors << v1(0), v2(0), 0.0,
	           v1(1), v2(1), 0.0,
	             0.0,   0.0, 0.0;
}


