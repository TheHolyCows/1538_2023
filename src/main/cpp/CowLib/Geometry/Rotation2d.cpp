#include "Rotation2d.h"

#include "Translation2d.h"

namespace CowLib
{
    Rotation2d::Rotation2d(double degrees, bool normalize)
    {
        if (normalize)
        {
            degrees = WrapDegrees(degrees);
        }

        m_Degrees = degrees;
    }

    Rotation2d::Rotation2d(double x, double y, bool normalize)
    {
        if (normalize)
        {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
            // we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = std::hypot(x, y);
            if (magnitude > 1E-9)
            {
                m_SinAngle = y / magnitude;
                m_CosAngle = x / magnitude;
            }
            else
            {
                m_SinAngle = 0.0;
                m_CosAngle = 1.0;
            }
        }
        else
        {
            m_CosAngle = x;
            m_SinAngle = y;
        }
    }

    Rotation2d::Rotation2d(const Rotation2d &other)
    {
        m_CosAngle = other.m_CosAngle;
        m_SinAngle = other.m_SinAngle;
        m_Degrees  = other.m_Degrees;
    }

    Rotation2d::Rotation2d(const frc::Rotation2d &other)
    {
        m_CosAngle = other.Cos();
        m_SinAngle = other.Sin();
        m_Degrees  = other.Degrees().value();
    }

    Rotation2d::Rotation2d(const Translation2d direction, bool normalize)
    {
        Rotation2d(direction.X(), direction.Y(), normalize);
    }

    bool Rotation2d::HasTrig() const
    {
        return !std::isnan(m_SinAngle) && !std::isnan(m_CosAngle);
    }

    bool Rotation2d::HasDegrees() const
    {
        return !std::isnan(m_Degrees);
    }

    void Rotation2d::EnsureTrigComputed() const
    {
        if (!HasTrig())
        {
            m_SinAngle = sin(m_Degrees * M_PI / 180.0);
            m_CosAngle = cos(m_Degrees * M_PI / 180.0);
        }
    }

    void Rotation2d::EnsureDegreesComputed() const
    {
        if (!HasDegrees())
        {
            m_Degrees = atan2(m_SinAngle, m_CosAngle) * 180.0 / M_PI;
        }
    }

    double Rotation2d::Cos() const
    {
        EnsureTrigComputed();
        return m_CosAngle;
    }

    double Rotation2d::Sin() const
    {
        EnsureTrigComputed();
        return m_SinAngle;
    }

    double Rotation2d::Tan() const
    {
        EnsureTrigComputed();
        if (fabs(m_CosAngle) < 1E-9)
        {
            if (m_SinAngle >= 0.0)
            {
                return std::numeric_limits<double>::max();
            }
            else
            {
                return std::numeric_limits<double>::min();
            }
        }
        return m_SinAngle / m_CosAngle;
    }

    double Rotation2d::GetDegrees() const
    {
        EnsureDegreesComputed();
        return m_Degrees;
    }

    Rotation2d Rotation2d::NearestPole() const
    {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (fabs(m_CosAngle) > fabs(m_SinAngle))
        {
            pole_cos = CowLib::sgn(m_CosAngle);
            pole_sin = 0.0;
        }
        else
        {
            pole_cos = 0.0;
            pole_sin = CowLib::sgn(m_SinAngle);
        }
        return Rotation2d(pole_cos, pole_sin, false);
    }

    double Rotation2d::GetRadians() const
    {
        return GetDegrees() * M_PI / 180.0;
    }

    Rotation2d Rotation2d::operator-() const
    {
        return Rotation2d(-m_Degrees, true);
    }

    Rotation2d Rotation2d::operator-(const Rotation2d &other) const
    {
        auto negativeOther = -other;
        return RotateBy(negativeOther);
    }

    Rotation2d Rotation2d::operator*(double scalar) const
    {
        return Rotation2d(m_Degrees * scalar, true);
    }

    Rotation2d Rotation2d::operator=(const Rotation2d &other) const
    {
        return Rotation2d(other);
    }

    Rotation2d Rotation2d::RotateBy(const Rotation2d &other) const
    {
        if (HasTrig() && other.HasTrig())
        {
            return Rotation2d(m_CosAngle * other.m_CosAngle - m_SinAngle * other.m_SinAngle,
                              m_CosAngle * other.m_SinAngle + m_SinAngle * other.m_CosAngle,
                              true);
        }
        else
        {
            return FromDegrees(GetDegrees() + other.GetDegrees());
        }
    }

    Rotation2d Rotation2d::Mirror() const
    {
        return FromDegrees(-m_Degrees);
    }

    Rotation2d Rotation2d::Normal() const
    {
        if (HasTrig())
        {
            return Rotation2d(-m_SinAngle, m_CosAngle, false);
        }
        else
        {
            return FromDegrees(GetDegrees() - 90.0);
        }
    }

    Rotation2d Rotation2d::Inverse() const
    {
        if (HasTrig())
        {
            printf("Inverse trig\n");
            return Rotation2d(m_CosAngle, -m_SinAngle, false);
        }
        else
        {
            printf("Inverse degrees\n");
            return Rotation2d::FromDegrees(-GetDegrees());
        }
    }

    Rotation2d Rotation2d::Flip() const
    {
        if (HasTrig())
        {
            return Rotation2d(-m_CosAngle, -m_SinAngle, false);
        }
        else
        {
            return FromDegrees(GetDegrees() + 180.0);
        }
    }

    bool Rotation2d::IsParallel(const Rotation2d &other) const
    {
        if (HasDegrees() && other.HasDegrees())
        {
            return EpsilonEquals(m_Degrees, other.m_Degrees)
                   || EpsilonEquals(m_Degrees, WrapDegrees(other.m_Degrees + 180.0));
        }
        else if (HasTrig() && other.HasTrig())
        {
            return EpsilonEquals(m_SinAngle, other.m_SinAngle) && EpsilonEquals(m_CosAngle, other.m_CosAngle);
        }
        else
        {
            // Use public, checked version.
            return EpsilonEquals(GetDegrees(), other.GetDegrees())
                   || EpsilonEquals(m_Degrees, WrapDegrees(other.m_Degrees + 180));
        }
    }

    Translation2d Rotation2d::ToTranslation() const
    {
        EnsureTrigComputed();
        return Translation2d(m_CosAngle, m_SinAngle);
    }

    Rotation2d Rotation2d::Interpolate(const Rotation2d &other, double x) const
    {
        if (x <= 0.0)
        {
            return Rotation2d(*this);
        }
        else if (x >= 1.0)
        {
            return Rotation2d(other);
        }
        double angle_diff = Inverse().RotateBy(other).GetDegrees();
        auto rotation     = Rotation2d::FromDegrees(angle_diff * x);
        return RotateBy(rotation);
    }

    double Rotation2d::Distance(const Rotation2d &other) const
    {
        return Inverse().RotateBy(other).GetDegrees();
    }

    Rotation2d Rotation2d::operator+(const Rotation2d &other) const
    {
        return RotateBy(other);
    }

    bool Rotation2d::operator==(const Rotation2d &other) const
    {
        return Distance(other) < 1E-9;
    }

    Rotation2d Rotation2d::GetRotation() const
    {
        return *this;
    }

} // namespace CowLib
