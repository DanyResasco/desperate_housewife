#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>


class point
{
public:
    double x;
    double y;
    double z;
};
 
class line
{
public:
    point p1;
    point p2;

};

double dot(point c1, point c2)
{
    return (c1.x * c2.x + c1.y * c2.y + c1.z * c2.z);
}
 
double norm(point c1)
{
    return std::sqrt(dot(c1, c1));
}
 
double getShortestDistance(line line1, line line2)
{
    double EPS = 0.00000001;
 
    point delta21 = new point();
    delta21.x = line1.p2.x - line1.p1.x;
    delta21.y = line1.p2.y - line1.p1.y;
    delta21.x = line1.p2.z - line1.p1.z;
 
    point delta43 = new point();
    delta43.x = line2.p2.x - line2.p1.x;
    delta43.y = line2.p2.y - line2.p1.y;
    delta43.z = line2.p2.z - line2.p1.z;
 
    point delta13 = new point();
    delta13.x = line1.p1.x - line2.p1.x;
    delta13.y = line1.p1.y - line2.p1.y;
    delta13.z = line1.p1.z - line2.p1.z;
 
    double a = dot(delta21, delta21);
    double b = dot(delta21, delta43);
    double c = dot(delta43, delta43);
    double d = dot(delta21, delta13);
    double e = dot(delta43, delta13);
    double D = a * c - b * b;
 
    double sc, sN, sD = D;
    double tc, tN, tD = D;
 
    if (D < EPS)
    {
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
 
    point dP = new point();
    dP.x = delta13.x + (sc * delta21.x) - (tc * delta43.x);
    dP.y = delta13.y + (sc * delta21.y) - (tc * delta43.y);
    dP.z = delta13.z + (sc * delta21.z) - (tc * delta43.z);
 
    return std::sqrt(dot(dP, dP));
}
 

#include <ros/ros.h>
#include <ros/console.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "TestLines");
  BasicGeometriesNode node;
  ROS_INFO("[TestLines] Node is ready");

  double spin_rate = 10;
  ros::param::get("~spin_rate",spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  point point1, point2, point3, point4;
  line line1, line2;
  point1.x = ; point1.y = ; point1.z = ;
  point2.x = ; point2.y = ; point2.z = ;
  point3.x = ; point3.y = ; point3.z = ;
  point4.x = ; point4.y = ; point4.z = ;

  line1.P1 = point1; line1.P2 = point2; 
  line2.P1 = point3; line2.P2 = point4; 

  getShortestDistance();

  ros::Rate rate(spin_rate); 

  // while (node.nh.ok())
  // {
    ros::spinOnce(); 
    rate.sleep();
  // }
  return 0;
}
