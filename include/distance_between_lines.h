#ifndef DISTANCE_BETWEEN_LINES_H
#define DISTANCE_BETWEEN_LINES_H

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <algorithm>

class LineCollisions
{

public:
  LineCollisions(){};
  ~LineCollisions(){};

  typedef Eigen::Vector3d Point;
  struct Line{
    Point P1, P2;
    double norm;
    Line(){};
    Line(Point P1_in, Point P2_in){P1=P1_in; P2=P2_in; norm = (P2-P1).norm();};
    bool operator<(const Line& other)
    {
      return norm < other.norm;
    }
  };

  double getPointDistance(Point P1, Point P2){return (P1 - P2).norm();};
  Line getClosestPoints(
    Line line1, Line line2);

};

    // This part computer the minimum distance between extremal point
    // std::list<LineCollisions::Line> list_lines;
    // std::vector<LineCollisions::Line> v_dist{ LineCollisions::Line(L1.P1, L2.P1),
    //                                      LineCollisions::Line(L1.P1, L2.P2),
    //                                      LineCollisions::Line(L1.P2, L2.P1),
    //                                      LineCollisions::Line(L1.P2, L2.P2) };
    // std::vector< LineCollisions::Line >::iterator result = std::min_element(std::begin(v_dist), std::end(v_dist));
    // line_out = v_dist[ std::distance(std::begin(v_dist), result) ];
    // return line_out;

/*
LineCollisions::Line LineCollisions::getClosestPoints(LineCollisions::
  Line line1, LineCollisions::Line line2)
{
    double EPS = 0.00000001;
 
    Point delta21 = line1.P2 - line1.P1;
    Point delta43 = line2.P2 - line2.P1;
    Point delta13 = line1.P1 - line2.P1;
 
    double a = delta21.adjoint() * (delta21); //1
    double b = delta21.adjoint() * (delta43);
    double c = delta43.adjoint() * (delta43); //1
    double d = delta21.adjoint() * (delta13);
    double e = delta43.adjoint() * (delta13);
    double D = a * c - b * b;
 
    double sc, sN, sD = D;
    double tc, tN, tD = D;
 
    if (D < EPS) // Lines are parallel
    {
        // std::cout << "Lines are parallel" << std::endl;
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else
    {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0)
        {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }
 
    if (tN < 0.0)
    {
        tN = 0.0;
 
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else
        {
            sN = (-d + b);
            sD = a;
        }
    }
 
    if (std::abs(sN) < EPS) sc = 0.0;
    else sc = sN / sD;
    if (std::abs(tN) < EPS) tc = 0.0;
    else tc = tN / tD;

    Line line_out( line1.P1 + (sc * delta21), line2.P1 + (tc * delta43));
    return line_out;
}*/


#endif