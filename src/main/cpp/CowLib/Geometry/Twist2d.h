#ifndef __COWLIB_GEOMETRY_TWIST2D_H__
#define __COWLIB_GEOMETRY_TWIST2D_H__

#include "../Utility.h"

#include <cmath>

namespace CowLib
{
    /**
     * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
     * new RigidTransform2d's from a Twist2d and visa versa.
     * <p>
     * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
     */
    class Twist2d
    {
    private:
    public:
        static Twist2d Identity() { return Twist2d(0, 0, 0); }

        const double dx;

        const double dy;

        const double dtheta; // Degrees

        Twist2d(double dx, double dy, double dtheta)
            : dx(dx),
              dy(dy),
              dtheta(dtheta)
        {
        }

        Twist2d operator*(double scalar) { return Twist2d(dx * scalar, dy * scalar, dtheta * scalar); }

        double Norm() const
        {
            // Common case of dy == 0
            if (dy == 0.0)
                return fabs(dx);
            return std::hypot(dx, dy);
        }

        double Curvature() const
        {
            if (fabs(dtheta) < 1E-9 && Norm() < 1E-9)
                return 0.0;
            return dtheta / Norm();
        }

        bool EpsilonEquals(const Twist2d &other) const
        {
            return ::CowLib::EpsilonEquals(dx, other.dx) && ::CowLib::EpsilonEquals(dy, other.dy)
                   && ::CowLib::EpsilonEquals(dtheta, other.dtheta);
        }
    };

} // namespace CowLib

#endif /* __COWLIB_GEOMETRY_TWIST2D_H__ */