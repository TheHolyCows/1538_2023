#ifndef __COWLIB_GEOMETRY_ROTATION2D_H__
#define __COWLIB_GEOMETRY_ROTATION2D_H__

#include "../Utility.h"

#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <limits>
#include <optional>

namespace CowLib
{
    class Translation2d;

    /**
    * A rotation in a 2d coordinate frame represented a point on the unit circle
    * (cosine and sine).
    * <p>
    * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
    */
    class Rotation2d
    {
    private:
        bool HasTrig() const;

        bool HasDegrees() const;

        void EnsureTrigComputed() const;

        void EnsureDegreesComputed() const;

    protected:
        mutable std::optional<double> m_CosAngle;
        mutable std::optional<double> m_SinAngle;
        mutable std::optional<double> m_Degrees;

        Rotation2d(double x, double y, double degrees)
        {
            m_CosAngle = x;
            m_SinAngle = y;
            m_Degrees  = degrees;
        }

        constexpr double WrapDegrees(const double degrees) const
        {
            auto res = std::fmod(degrees, 360.0);
            res      = std::fmod(res + 360.0, 360.0);

            if (res > 180.0)
            {
                res -= 360.0;
            }

            return res;
        }

    public:
        static Rotation2d identity() { return Rotation2d(); }

        // Rotation2d operator=(const Rotation2d &other) const;

        Rotation2d() { Rotation2d(1.0, 0.0, 0.0); }

        Rotation2d(double degrees, bool normalize);

        Rotation2d(double x, double y, bool normalize);

        // Rotation2d(const Rotation2d &other);

        Rotation2d(const frc::Rotation2d &other);

        Rotation2d(const Translation2d direction, bool normalize);

        static Rotation2d FromRadians(double angle_radians) { return FromDegrees(angle_radians * 180.0 / M_PI); }

        static Rotation2d FromDegrees(double angle_degrees) { return Rotation2d(angle_degrees, true); }

        double Cos() const;

        double Sin() const;

        double Tan() const;

        double GetDegrees() const;

        /**
        * Based on Team 1323's method of the same name.
        *
        * @return Rotation2d representing the angle of the nearest axis to the angle in standard position
        */
        Rotation2d NearestPole() const;

        double GetRadians() const;

        Rotation2d operator-() const;

        Rotation2d operator-(const Rotation2d &other) const;

        Rotation2d operator*(double scalar) const;

        /**
        * We can rotate this Rotation2d by adding together the effects of it and
        * another rotation.
        *
        * @param other The other rotation. See:
        *              https://en.wikipedia.org/wiki/Rotation_matrix
        * @return This rotation rotated by other.
        */
        Rotation2d RotateBy(const Rotation2d &other) const;

        Rotation2d Mirror() const;

        Rotation2d Normal() const;

        /**
        * The inverse of a Rotation2d "undoes" the effect of this rotation.
        *
        * @return The inverse of this rotation.
        */
        Rotation2d Inverse() const;

        /**
        * Obtain a Rotation2d that points in the opposite direction from this rotation.
        * @return This rotation rotated by 180 degrees.
        */
        Rotation2d Flip() const;

        bool IsParallel(const Rotation2d &other) const;

        Translation2d ToTranslation() const;

        Rotation2d Interpolate(const Rotation2d &other, double x) const;

        double Distance(const Rotation2d &other) const;

        Rotation2d operator+(const Rotation2d &other) const;

        bool operator==(const Rotation2d &other) const;

        Rotation2d GetRotation() const;
    };

} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_ROTATION2D_H__ */