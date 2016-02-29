
#ifndef KUKA_LWR_UTILITIES_H
#define KUKA_LWR_UTILITIES_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include "distance_between_lines.h"

inline void saurateJointPositions( KDL::JntArray &q, double soft_limit)
{

    const double deg2rad = M_PI  / 180.0;
    q(0) = (std::abs(q(0)) >= ((170.0 * deg2rad) - soft_limit) ?
            std::copysign( ((170.0 * deg2rad) - soft_limit), q(0))
            : q(0));
    q(1) = (std::abs(q(1)) >= ((120.0 * deg2rad) - soft_limit) ?
            std::copysign(((120.0 * deg2rad) - soft_limit), q(1))
            : q(1));
    q(2) = (std::abs(q(2)) >= ((170.0 * deg2rad) - soft_limit) ?
            std::copysign(((170.0 * deg2rad) - soft_limit), q(2))
            : q(2));
    q(3) = (std::abs(q(3)) >= ((120.0 * deg2rad) - soft_limit) ?
            std::copysign(((120.0 * deg2rad) - soft_limit), q(3))
            : q(3));
    q(4) = (std::abs(q(4)) >= ((170.0 * deg2rad) - soft_limit) ?
            std::copysign(((170.0 * deg2rad) - soft_limit), q(4))
            : q(4));
    q(5) = (std::abs(q(5)) >= ((120.0 * deg2rad) - soft_limit) ?
            std::copysign(((120.0 * deg2rad) - soft_limit), q(5))
            : q(5));
    q(6) = (std::abs(q(6)) >=  ((170.0 * deg2rad) - soft_limit) ?
            std::copysign( ((170.0 * deg2rad) - soft_limit), q(6))
            : q(6));
}

inline void saurateJointVelocities( KDL::JntArray &qp, double percentage = 0.7)
{

    const double deg2rad = M_PI  / 180.0;

    qp(0) = (std::abs(qp(0)) >= 110.0 * deg2rad * percentage ?
                                         std::copysign(110.0 * deg2rad * percentage, qp(0))
                                         : qp(0));
    qp(1) = (std::abs(qp(1)) >= 110.0 * deg2rad * percentage ?
                                         std::copysign(110.0 * deg2rad * percentage, qp(1))
                                         : qp(1));
    qp(2) = (std::abs(qp(2)) >= 128.0 * deg2rad * percentage ?
                                         std::copysign(128.0 * deg2rad * percentage, qp(2))
                                         : qp(2));
    qp(3) = (std::abs(qp(3)) >= 128.0 * deg2rad * percentage ?
                                         std::copysign(128.0 * deg2rad * percentage, qp(3))
                                         : qp(3));
    qp(4) = (std::abs(qp(4)) >= 204.0 * deg2rad * percentage ?
                                         std::copysign(204.0 * deg2rad * percentage, qp(4))
                                         : qp(4));
    qp(5) = (std::abs(qp(5)) >= 184.0 * deg2rad * percentage ?
                                         std::copysign(184.0 * deg2rad * percentage, qp(5))
                                         : qp(5));
    qp(6) = (std::abs(qp(6)) >= 184.0 * deg2rad * percentage ?
                                         std::copysign(184.0 * deg2rad * percentage, qp(6))
                                         : qp(6));
}


Eigen::Matrix<double, 3, 3> inline getVectorHat(Eigen::Matrix<double, 3, 1> vector_in)
{
    Eigen::Matrix<double, 3, 3> vector_hat = Eigen::Matrix<double, 3, 3>::Zero();

    vector_hat << 0, -vector_in(2, 0), vector_in(1, 0),
               vector_in(2, 0), 0, -vector_in(0, 0),
               -vector_in(1, 0), vector_in(0, 0), 0;
    return vector_hat;
}

Eigen::Matrix<double, 6, 6> inline getAdjointT( KDL::Frame Frame_in)
{
    Eigen::Matrix<double, 6, 6> Adjoint_local = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 3> rotation_local(Frame_in.M.data);
    Eigen::Matrix<double, 3, 1> position_local(Frame_in.p.data);

    Adjoint_local.block<3, 3>(0, 0) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 3) = rotation_local.transpose();
    Adjoint_local.block<3, 3>(3, 0) = -rotation_local.transpose() * getVectorHat(position_local);

    return Adjoint_local;
}


void inline getClosestPointstoCylinder( KDL::Frame T_link_object, KDL::Frame &T_CollisionPoint, double radius, double height)
{
    LineCollisions LineCollisionsLocal;

    LineCollisions::Point Point1(T_link_object.p.x(), T_link_object.p.y(), T_link_object.p.z());
    LineCollisions::Point Point2(T_link_object.p.x() + .001, T_link_object.p.y() + .001, T_link_object.p.z() + .001);

    LineCollisions::Point Point3(0.0, 0.0, height / 2.0);
    LineCollisions::Point Point4(0.0, 0.0, -height / 2.0);

    LineCollisions::Line Line1(Point1, Point2);
    LineCollisions::Line Line2(Point3, Point4);
    LineCollisions::Line ClosestPoints;

    ClosestPoints = LineCollisionsLocal.getClosestPoints(Line1, Line2);

    KDL::Vector V1(ClosestPoints.P1[0], ClosestPoints.P1[1], ClosestPoints.P1[2]);
    KDL::Vector V2(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2]);
    KDL::Vector V3(V1 - V2);
    // V3  = (V1 - V2);
    KDL::Frame pos_final;
    KDL::Vector collision_point_on_line(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2] );
    if ( (ClosestPoints.P2 == Point4) || (ClosestPoints.P2 == Point3) )
    {   // The point is on one of the planar face 

        KDL::Vector V4( V3.x(), V3.y(), 0.0  );
        double pos_on_plane = V4.Norm();
        if (pos_on_plane >= radius )
        { // The point is on the border
            KDL::Vector Uz(V4 / V4.Norm() );
            KDL::Vector Ux(0.0, 0.0, 1.0);
            pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector( 0.0 , 0.0, radius) );
            KDL::Vector V5;
            V5 = (T_link_object.Inverse() * pos_final).Inverse().p;
            V5 = V5 /  V5.Norm();
            double angle = std::atan2(V5.x(), V5.z());
            pos_final.M =  pos_final.M * KDL::Rotation::RotY( angle );
        }
        else
        {   //The cylinder is on the one of the planar faces
            KDL::Vector Ux;
            KDL::Vector Uz;
            Ux = ( V4 / V4.Norm() );
            Uz = KDL::Vector(0.0, 0.0, 1.0);
            if ( ClosestPoints.P2 == Point4 )
            {
                Uz = KDL::Vector(0.0, 0.0, -1.0);
            }
            pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(pos_on_plane, 0.0, 0.0) );

        }
    }
    else
    {   // The point is on the cylindrical face
        KDL::Vector Uz(V3 / V3.Norm() );
        KDL::Vector Ux(0.0, 0.0, 1.0);
        pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                    KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(0.0, 0.0, radius) );
    }

    T_CollisionPoint.M = pos_final.M;
    T_CollisionPoint.p = pos_final.p;

}

double inline VelocityLimit(KDL::Twist x_dot_d, double limit = 1.0)
{
    Eigen::Matrix<double, 3, 1> x_dot_d_local = Eigen::Matrix<double, 3, 1>::Zero();
    x_dot_d_local << x_dot_d.vel.data[0], x_dot_d.vel.data[1], x_dot_d.vel.data[2];
    double temp = limit / std::sqrt(x_dot_d_local.transpose() * x_dot_d_local);

    return std::min(1.0, temp);
}




#endif