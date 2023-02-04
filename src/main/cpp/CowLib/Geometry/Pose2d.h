#ifndef __COWLIB_GEOMETRY_POSE2D_H__
#define __COWLIB_GEOMETRY_POSE2D_H__

#include "Rotation2d.h"

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

        ~Pose2d() = default;

        double GetX() const { return m_X; }

        double GetY() const { return m_Y; }

        Rotation2d GetRotation() const { return m_Rotation; }
    };

} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_POSE2D_H__ */