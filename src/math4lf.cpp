
#include <ros/ros.h>
#include <Eigen/Dense> // for EigenSolver
#include "mmath/math4lf.h"

Eigen::Vector2d getLine(std::vector<Point2D> poins)
{
	int n = poins.size();
	if(!n) return Eigen::Vector2d::Zero(2);
	double cov  = 0.0;
	double varx = 0.0;
	double vary = 0.0;
	double x_sum = 0.0;
	double y_sum = 0.0;
	double x_ave = 0.0;
	double y_ave = 0.0;
	double prodsum_xy = 0.0;
	double prodsum_xx = 0.0;
	double prodsum_yy = 0.0;
	Eigen::Matrix2d vcov;

	for(auto itr = poins.begin(); itr != poins.end(); ++itr){
		x_sum += itr->x;
		y_sum += itr->y;
		prodsum_xx += itr->x * itr->x;
		prodsum_yy += itr->y * itr->y;
		prodsum_xy += itr->x * itr->y;
	}
	x_ave = x_sum / n;
	y_ave = y_sum / n;

	cov  = prodsum_xy / n - x_ave * y_ave;
	varx = prodsum_xx / n - x_ave * x_ave;
	vary = prodsum_yy / n - y_ave * y_ave;

	vcov << varx, cov,
		 	cov, vary;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	Eigen::Vector2d values  = es.eigenvalues().real();
	Eigen::Matrix2d vectors = es.eigenvectors().real();

	Eigen::Vector2d vec1;
	if(values(0) < values(1)){
		vec1 = vectors.col(1);
	}else{
		vec1 = vectors.col(0);
	}

	return vec1;
}

void getLine(const std::vector<Point2D> &poins, Eigen::Vector2d &slope, Eigen::Vector2d &center)
{
	int n = poins.size();
	if(!n){
		std::cerr << "POINT SIZE ZERO! in getLine" << std::endl;
		return;
	}
	double cov  = 0.0;
	double varx = 0.0;
	double vary = 0.0;
	double x_sum = 0.0;
	double y_sum = 0.0;
	double x_ave = 0.0;
	double y_ave = 0.0;
	double prodsum_xy = 0.0;
	double prodsum_xx = 0.0;
	double prodsum_yy = 0.0;
	Eigen::Matrix2d vcov;

	for(auto itr = poins.begin(); itr != poins.end(); ++itr){
		x_sum += itr->x;
		y_sum += itr->y;
		prodsum_xx += itr->x * itr->x;
		prodsum_yy += itr->y * itr->y;
		prodsum_xy += itr->x * itr->y;
	}
	x_ave = x_sum / n;
	y_ave = y_sum / n;

	cov  = prodsum_xy / n - x_ave * y_ave;
	varx = prodsum_xx / n - x_ave * x_ave;
	vary = prodsum_yy / n - y_ave * y_ave;

	vcov << varx, cov,
		 	cov, vary;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	Eigen::Vector2d values  = es.eigenvalues().real();
	Eigen::Matrix2d vectors = es.eigenvectors().real();

	if(values(0) < values(1)){
		slope = vectors.col(1);
	}else{
		slope = vectors.col(0);
	}

	center << x_ave, y_ave;
}

void getLine(const std::vector<Point2D> &poins, Eigen::Vector2d &slope, double &curv)
{
	int n = poins.size();
	if(!n){
		std::cerr << "POINT SIZE ZERO! in getLine" << std::endl;
		curv = 1;
		return;
	}
	double cov  = 0.0;
	double varx = 0.0;
	double vary = 0.0;
	double x_sum = 0.0;
	double y_sum = 0.0;
	double x_ave = 0.0;
	double y_ave = 0.0;
	double prodsum_xy = 0.0;
	double prodsum_xx = 0.0;
	double prodsum_yy = 0.0;
	Eigen::Matrix2d vcov;

	for(auto itr = poins.begin(); itr != poins.end(); ++itr){
		x_sum += itr->x;
		y_sum += itr->y;
		prodsum_xx += itr->x * itr->x;
		prodsum_yy += itr->y * itr->y;
		prodsum_xy += itr->x * itr->y;
	}
	x_ave = x_sum / n;
	y_ave = y_sum / n;

	cov  = prodsum_xy / n - x_ave * y_ave;
	varx = prodsum_xx / n - x_ave * x_ave;
	vary = prodsum_yy / n - y_ave * y_ave;

	vcov << varx, cov,
		 	cov, vary;

	Eigen::EigenSolver<Eigen::Matrix2d> es(vcov);
	Eigen::Vector2d values  = es.eigenvalues().real();
	Eigen::Matrix2d vectors = es.eigenvectors().real();
	double lambda_min;

	if(values(0) < values(1)){
		lambda_min = values(0);
		slope = vectors.col(1);
	}else{
		lambda_min = values(1);
		slope = vectors.col(0);
	}

	if(values(0) + values(1)) curv = 2.0 * lambda_min / ( values(0) + values(1) );
}

Eigen::Vector2d getIntersection(const std::vector<Point2D> &line1, const std::vector<Point2D> &line2)
{
	Eigen::Vector2d point;
	Eigen::Vector2d c1, c2, x1, x2;
	double x1y2, x2y1, det;

	getLine(line1, x1, c1);
	getLine(line2, x2, c2);

	x1y2 = x1(0) * x2(1);
	x2y1 = x2(0) * x1(1);
	det = x1y2 - x2y1;

	if(fabs(det) < 1e-5){
		point << 0, 0;
	}else{
		point << -( x2y1*c1(0) - x1(0)*x2(0)*(c1(1)-c2(1)) - x1y2*c2(0) ) / det,
				  ( x1y2*c1(1) - x1(1)*x2(1)*(c1(0)-c2(0)) - x2y1*c2(1) ) / det;
	}

	return point;
}

double getSlope(Point2D &begin, Point2D &end)
{
	return atan2(begin.y - end.y, begin.x - end.x);
}

double getDist(std::vector<Point2D> &points)
{
	if(points.size()){
		return sqrt(pow(points.begin()->x - points.back().x, 2) + pow(points.begin()->y - points.back().y, 2));
	}else{
		return 0.0;
	}
}

double getDist(Point2D &begin, Point2D &end)
{
	return sqrt(pow(begin.x - end.x, 2) + pow(begin.y - end.y, 2));
}

double getDist2Line(std::vector<Point2D> &line)
{
	Eigen::Vector2d point;
	Eigen::Vector2d x, c, n;
	double det;
	double dist = 1e4;

	getLine(line, x, c);

	n << -x(1), x(0);

	det = n(0) * x(1) - n(1) * x(0);

	if(det) dist = (c(0)*x(1) - c(1)*x(0)) / det;

	return fabs(dist);
}

void thetaAround(double &theta)
{
	while(theta >  M_PI) theta -= 2*M_PI;
	while(theta < -M_PI) theta += 2*M_PI;
}

void theta2slope(double &theta)
{
	if(theta >  M_PI/2) theta -= M_PI;
	if(theta < -M_PI/2) theta += M_PI;
}

void theta2vec(const double &theta, Eigen::Vector2d &vec)
{
	vec << cos(theta), sin(theta); 
}

void vec2theta(const Eigen::Vector2d &v, double &theta)
{
	theta = atan2(v(1), v(0));
}

bool isTurn(const double &previous, const double &current)
{
	double diff = previous - current;

	return fabs(M_PI - fabs(diff)) < M_PI / 2.5 ? true : false;
}

