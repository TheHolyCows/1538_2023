#include "Translation2d.h"

#include "Rotation2d.h"

namespace CowLib
{
    const Translation2d Translation2d::Identity()
    {
        return Translation2d();
    }

    Translation2d Translation2d::operator=(const Translation2d other) const
    {
        return Translation2d(other.m_X, other.m_Y);
    }

    double Translation2d::Norm() const
    {
        return std::hypot(m_X, m_Y);
    }

    double Translation2d::NormSquared() const
    {
        return m_X * m_X + m_Y * m_Y;
    }

    double Translation2d::X() const
    {
        return m_X;
    }

    double Translation2d::Y() const
    {
        return m_Y;
    }

    /**
        * We can compose Translation2d's by adding together the x and y shifts.
        *
        * @param other The other translation to add.
        * @return The combined effect of translating by this object and the other.
        */
    Translation2d Translation2d::TranslateBy(const Translation2d &other) const
    {
        return Translation2d(m_X + other.m_X, m_Y + other.m_Y);
    }

    Translation2d Translation2d::operator+(const Translation2d &other) const
    {
        return TranslateBy(other);
    }

    Translation2d Translation2d::operator-(const Translation2d &other) const
    {
        return Translation2d(m_X - other.m_X, m_Y - other.m_Y);
    }

    Translation2d Translation2d::operator-() const
    {
        return Translation2d(-m_X, -m_Y);
    }

    Translation2d Translation2d::operator*(double scalar) const
    {
        return Translation2d(m_X * scalar, m_Y * scalar);
    }

    /**
        * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
        *
        * @param rotation The rotation to apply.
        * @return This translation rotated by rotation.
        */
    Translation2d Translation2d::RotateBy(Rotation2d &rotation) const
    {
        return Translation2d(m_X * rotation.Cos() - m_Y * rotation.Sin(), m_X * rotation.Sin() + m_Y * rotation.Cos());
    }

    Rotation2d Translation2d::Direction() const
    {
        return Rotation2d(m_X, m_Y, true);
    }

    /**
        * The inverse simply means a Translation2d that "undoes" this object.
        *
        * @return Translation by -x and -y.
        */
    Translation2d Translation2d::Inverse() const
    {
        return Translation2d(-m_X, -m_Y);
    }

    Translation2d Translation2d::Interpolate(const Translation2d other, double x) const
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

    Translation2d Translation2d::Extrapolate(const Translation2d other, double x) const
    {
        return Translation2d(x * (other.m_X - m_X) + m_X, x * (other.m_Y - m_Y) + m_Y);
    }

    Translation2d Translation2d::Scale(double s)
    {
        return Translation2d(m_X * s, m_Y * s);
    }

    bool Translation2d::EpsilonEquals(const Translation2d &other) const
    {
        constexpr double epsilon = 1E-9;
        return fabs(m_X - other.m_X) < epsilon && fabs(m_Y - other.m_Y) < epsilon;
    }

    bool Translation2d::operator==(const Translation2d other) const
    {
        return EpsilonEquals(other);
    }

    double Translation2d::Dot(const Translation2d &a, const Translation2d &b)
    {
        return a.m_X * b.m_X + a.m_Y * b.m_Y;
    }

    Rotation2d Translation2d::GetAngle(const Translation2d &a, const Translation2d &b)
    {
        double cos_angle = Dot(a, b) / (a.Norm() * b.Norm());
        if (std::isnan(cos_angle))
        {
            return Rotation2d();
        }
        return Rotation2d::FromRadians(acos(LimitMix(cos_angle)));
    }

    double Translation2d::Cross(const Translation2d &a, const Translation2d &b)
    {
        return a.m_X * b.m_Y - a.m_Y * b.m_X;
    }

    double Translation2d::Distance(const Translation2d &other) const
    {
        return Inverse().TranslateBy(other).Norm();
    }

    bool Translation2d::Equals(const Translation2d &other)
    {
        return Distance(other) < 1E-9;
    }

    Translation2d Translation2d::GetTranslation() const
    {
        return *this;
    }

} // namespace CowLib
