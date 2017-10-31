
#ifndef MATCHEY_MATH_H
#define MATCHEY_MATH_H

#include <Eigen/Core>

struct Point2D
{
	double x;
	double y;
};

// struct Line
// {
// 	double slope;
// 	double intercept;
// }

Eigen::Vector2d getLine(std::vector<Point2D>);
void getLine(const std::vector<Point2D>&, Eigen::Vector2d&, Eigen::Vector2d&); // , slope, center
void getLine(const std::vector<Point2D>&, Eigen::Vector2d&, double&); //points, slope, curv
Eigen::Vector2d getIntersection(const std::vector<Point2D>&, const std::vector<Point2D>&);
double getSlope(Point2D&, Point2D&);
double getDist(std::vector<Point2D>&);
double getDist(Point2D&, Point2D&);
double getDist2Line(std::vector<Point2D>&);
void thetaAround(double&); //  -> -PI ~ PI
void theta2slope(double&); // -PI~PI -> -PI/2 ~ PI/2
void theta2vec(const double&, Eigen::Vector2d&);
void vec2theta(const Eigen::Vector2d&, double&);
bool isTurn(const double&, const double&);

#endif

