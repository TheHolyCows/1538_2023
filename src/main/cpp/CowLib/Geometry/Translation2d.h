#ifndef __COWLIB_GEOMETRY_TRANSLATION2D_H__
#define __COWLIB_GEOMETRY_TRANSLATION2D_H__

#include <cmath>

// #include <frc/geometry/Translation2d.h>

namespace CowLib
{
    /**
    * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
    */
    class Translation2d
    {
    protected:
        double m_X = 0;
        double m_Y = 0;

    public:
        static const Translation2d Identity() { return Translation2d(); }

        constexpr Translation2d() = default;

        constexpr Translation2d(double x, double y)
        {
            m_X = x;
            m_Y = y;
        }

        constexpr Translation2d(const Translation2d &start, const Translation2d &end)
        {
            m_X = end.m_X - start.m_X;
            m_Y = end.m_Y - start.m_Y;
        }

        // constexpr Translation2d(const frc::Translation2d &other)
        // {
        //     m_X = other.X().convert<units::foot>().value();
        //     m_Y = other.Y().convert<units::foot>().value();
        // }

        constexpr Translation2d(const Translation2d &other)
        {
            m_X = other.m_X;
            m_Y = other.m_Y;
        }

        Translation2d &operator=(const Translation2d &other)
        {
            m_X = other.m_X;
            m_Y = other.m_Y;
            return *this;
        }

        double Norm() const { return std::hypot(m_X, m_Y); }

        double NormSquared() const { return m_X * m_X + m_Y * m_Y; }

        double X() const { return m_X; }

        double Y() const { return m_Y; }

        /**
        * We can compose Translation2d's by adding together the x and y shifts.
        *
        * @param other The other translation to add.
        * @return The combined effect of translating by this object and the other.
        */
        Translation2d TranslateBy(const Translation2d &other) const
        {
            return Translation2d(m_X + other.m_X, m_Y + other.m_Y);
        }

        Translation2d operator+(const Translation2d &other) const { return TranslateBy(other); }

        Translation2d operator-(const Translation2d &other) const
        {
            return Translation2d(m_X - other.m_X, m_Y - other.m_Y);
        }

        Translation2d operator-() const { return Translation2d(-m_X, -m_Y); }

        Translation2d operator*(double scalar) const { return Translation2d(m_X * scalar, m_Y * scalar); }

        /**
        * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
        *
        * @param rotation The rotation to apply.
        * @return This translation rotated by rotation.
        */
        // Translation2d RotateBy(const Rotation2d &rotation) const
        // {
        //     return Translation2d(m_X * rotation.Cos() - m_Y * rotation.Sin(),
        //                          m_X * rotation.Sin() + m_Y * rotation.Cos());
        // }
        // Translation2d rotateBy(final Rotation2d rotation)
        // {
        //     return new Translation2d(x_ * rotation.cos() - y_ * rotation.sin(),
        //                              x_ * rotation.sin() + y_ * rotation.cos());
        // }

        // public

        //     Rotation2d direction() { return new Rotation2d(x_, y_, true); }

        /**
        * The inverse simply means a Translation2d that "undoes" this object.
        *
        * @return Translation by -x and -y.
        */
        Translation2d Inverse() const { return Translation2d(-m_X, -m_Y); }

        Translation2d Interpolate(const Translation2d other, double x) const
        {
            if (x <= 0)
            {
                return Translation2d(*this);
            }
            else if (x >= 1)
            {
                return Translation2d(other);
            }

            return Extrapolate(other, x);
        }

        Translation2d Extrapolate(const Translation2d other, double x) const
        {
            return Translation2d(x * (other.m_X - m_X) + m_X, x * (other.m_Y - m_Y) + m_Y);
        }

        Translation2d Scale(double s) { return Translation2d(m_X * s, m_Y * s); }

        bool EpsilonEquals(const Translation2d &other) const
        {
            constexpr double epsilon = 1E-9;
            return fabs(m_X - other.m_X) < epsilon && fabs(m_Y - other.m_Y) < epsilon;
        }

        bool operator==(const Translation2d other) const;

        // std::string ToString()
        // {
        //     final DecimalFormat format = new DecimalFormat("#0.000");
        //     return "(" + format.format(x_) + "," + format.format(y_) + ")";
        // }

        // @Override public String toCSV()
        // {
        //     final DecimalFormat format = new DecimalFormat("#0.000");
        //     return format.format(x_) + "," + format.format(y_);
        // }

        static double Dot(const Translation2d &a, const Translation2d &b) { return a.m_X * b.m_X + a.m_Y * b.m_Y; }

        // static Rotation2d getAngle(final Translation2d a, final Translation2d b)
        // {
        //     double cos_angle = dot(a, b) / (a.norm() * b.norm());
        //     if (Double.isNaN(cos_angle))
        //     {
        //         return new Rotation2d();
        //     }
        //     return Rotation2d.fromRadians(Math.acos(Util.limit(cos_angle, 1.0)));
        // }

        static double Cross(const Translation2d &a, const Translation2d &b) { return a.m_X * b.m_Y - a.m_Y * b.m_X; }

        double Distance(const Translation2d &other) const { return Inverse().TranslateBy(other).Norm(); }

        bool Equals(const Translation2d &other) { return Distance(other) < 1E-9; }

        Translation2d GetTranslation() const { return *this; }
    };
} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_TRANSLATION2D_H__ */