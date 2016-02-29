
#include "distance_between_lines.h"

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
}