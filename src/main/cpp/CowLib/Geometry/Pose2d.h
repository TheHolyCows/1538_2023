#ifndef __COWLIB_GEOMETRY_POSE2D_H__
#define __COWLIB_GEOMETRY_POSE2D_H__

#include "Rotation2d.h"
#include "Translation2d.h"
#include "Twist2d.h"

#include <cmath>
#include <frc/geometry/Pose2d.h>

namespace CowLib
{
    class Pose2d
    {
    private:
        double m_X = 0;
        double m_Y = 0;
        Rotation2d m_Rotation;

    public:
        Pose2d() = default;

        Pose2d(double x, double y, double rotation)
        {
            m_X        = x;
            m_Y        = y;
            m_Rotation = Rotation2d::FromDegrees(rotation);
        }

        Pose2d(double x, double y, Rotation2d rotation)
        {
            m_X        = x;
            m_Y        = y;
            m_Rotation = rotation;
        }

        Pose2d(frc::Pose2d pose)
        {
            m_X        = pose.X().convert<units::foot>().value();
            m_Y        = pose.Y().convert<units::foot>().value();
            m_Rotation = Rotation2d(pose.Rotation());
        }

        ~Pose2d() = default;

        /**
        * Obtain a new Pose2d from a (constant curvature) velocity. See:
        * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
        */
        static Pose2d Exp(const Twist2d &delta)
        {
            double sin_theta = sin(delta.dtheta * M_PI / 180.0);
            double cos_theta = cos(delta.dtheta * M_PI / 180.0);
            double s, c;
            if (fabs(delta.dtheta) < 1E-9)
            {
                s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
                c = .5 * delta.dtheta;
            }
            else
            {
                s = sin_theta / delta.dtheta;
                c = (1.0 - cos_theta) / delta.dtheta;
            }
            return Pose2d(delta.dx * s - delta.dy * c,
                          delta.dx * c + delta.dy * s,
                          Rotation2d(cos_theta, sin_theta, true));
        }

        double GetX() const { return m_X; }

        double GetY() const { return m_Y; }

        Rotation2d GetRotation() const { return m_Rotation; }
    };

} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_POSE2D_H__ */