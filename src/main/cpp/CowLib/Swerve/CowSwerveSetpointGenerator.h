#ifndef __COWLIB_COW_SWERVE_SETPOINT_GENERATOR_H__
#define __COWLIB_COW_SWERVE_SETPOINT_GENERATOR_H__

#include "CowChassisSpeeds.h"
#include "CowSwerveKinematics.h"
#include "CowSwerveModuleState.h"

#include <array>
#include <frc/geometry/Rotation2d.h>
#include <functional>

namespace CowLib
{
    class CowSwerveSetpointGenerator
    {
    public:
        struct KinematicLimits
        {
            double maxDriveVelocity;
            double maxDriveAcceleration;
            double maxRotationalVelocity;
        };

        struct SwerveSetpoint
        {
            CowChassisSpeeds chassisSpeeds;
            std::array<CowSwerveModuleState, 4> moduleStates;
        };

    private:
        typedef std::function<double(double, double)> Function2d;

        CowSwerveKinematics *m_Kinematics;

        bool FlipHeading(frc::Rotation2d previousToGoal);
        double UnwrapAngle(double ref, double angle);

        double
        FindRoot(Function2d func, double x0, double y0, double f0, double x1, double y1, double f1, int iterationsLeft);

        double FindSteeringMaxS(double x0,
                                double y0,
                                double f0,
                                double x1,
                                double y1,
                                double f1,
                                double maxDeviation,
                                int maxIterations);

        double FindDriveMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double maxVelStep, int maxIterations);

        double SinDegrees(double angle);
        double CosDegrees(double angle);

        bool EpsilonEquals(double a, double b);

        template<typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

    public:
        CowSwerveSetpointGenerator(CowSwerveKinematics *kinematics);

        ~CowSwerveSetpointGenerator();

        SwerveSetpoint GenerateSetpoint(KinematicLimits limits,
                                        SwerveSetpoint currentSetpoint,
                                        CowChassisSpeeds desiredState,
                                        double dt);
    };
} // namespace CowLib

#endif /* __COWLIB_COW_SWERVE_SETPOINT_GENERATOR_H__ */