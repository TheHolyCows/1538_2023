#ifndef __COWLIB_GEOMETRY_TRANSLATION2D_H__
#define __COWLIB_GEOMETRY_TRANSLATION2D_H__

#include "stdio.h"

#include <cmath>
#include <frc/geometry/Translation2d.h>

namespace CowLib
{
    class Rotation2d;

    /**
    * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
    */
    class Translation2d
    {
    protected:
        double m_X = 0;
        double m_Y = 0;

    public:
        static const Translation2d Identity();

        // Translation2d() = default;

        constexpr Translation2d(double x = 0, double y = 0)
            : m_X(x),
              m_Y(y)
        {
        }

        constexpr Translation2d(const Translation2d &start, const Translation2d &end)
        {
            m_X = end.m_X - start.m_X;
            m_Y = end.m_Y - start.m_Y;
        }

        constexpr Translation2d(const frc::Translation2d &other)
        {
            m_X = other.X().convert<units::foot>().value();
            m_Y = other.Y().convert<units::foot>().value();
        }

        constexpr Translation2d(const Translation2d &other)
        {
            m_X = other.m_X;
            m_Y = other.m_Y;
        }

        // Translation2d operator=(const Translation2d other) const;

        // double Norm() const;

        // double NormSquared() const;

        double X() const;

        double Y() const;

        // /**
        // * We can compose Translation2d's by adding together the x and y shifts.
        // *
        // * @param other The other translation to add.
        // * @return The combined effect of translating by this object and the other.
        // */
        // Translation2d TranslateBy(const Translation2d &other) const;

        // Translation2d operator+(const Translation2d &other) const;

        // Translation2d operator-(const Translation2d &other) const;

        // Translation2d operator-() const;

        // Translation2d operator*(double scalar) const;

        // /**
        // * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
        // *
        // * @param rotation The rotation to apply.
        // * @return This translation rotated by rotation.
        // */
        // Translation2d RotateBy(Rotation2d &rotation) const;

        // Rotation2d Direction() const;

        // /**
        // * The inverse simply means a Translation2d that "undoes" this object.
        // *
        // * @return Translation by -x and -y.
        // */
        // Translation2d Inverse() const;

        // Translation2d Interpolate(const Translation2d other, double x) const;

        // Translation2d Extrapolate(const Translation2d other, double x) const;

        // Translation2d Scale(double s);

        // bool EpsilonEquals(const Translation2d &other) const;

        // bool operator==(const Translation2d other) const;

        // static double Dot(const Translation2d &a, const Translation2d &b);

        // static Rotation2d GetAngle(const Translation2d &a, const Translation2d &b);

        // static double Cross(const Translation2d &a, const Translation2d &b);

        // double Distance(const Translation2d &other) const;

        // bool Equals(const Translation2d &other);

        // Translation2d GetTranslation() const;
    };
} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_TRANSLATION2D_H__ */