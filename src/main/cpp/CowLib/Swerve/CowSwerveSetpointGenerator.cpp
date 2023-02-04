#include "CowSwerveSetpointGenerator.h"

#include "../Utility.h"
#include "CowChassisSpeeds.h"
#include "CowSwerveKinematics.h"
#include "units/angle.h"

#include <array>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <optional>

namespace CowLib
{
    CowSwerveSetpointGenerator::CowSwerveSetpointGenerator(CowSwerveKinematics *kinematics)
    {
        m_Kinematics = kinematics;
    }

    CowSwerveSetpointGenerator::~CowSwerveSetpointGenerator()
    {
    }

    CowSwerveSetpointGenerator::SwerveSetpoint
    CowSwerveSetpointGenerator::GenerateSetpoint(KinematicLimits limits,
                                                 SwerveSetpoint previousSetpoint,
                                                 CowChassisSpeeds desiredState,
                                                 double dt)
    {
        auto desiredModuleStates = m_Kinematics->CalculateModuleStates(desiredState, 0, 0);

        CowSwerveKinematics::DesaturateSpeeds(&desiredModuleStates, limits.maxDriveVelocity);
        desiredState = m_Kinematics->CalculateChassisSpeeds(desiredModuleStates);

        bool needToRotate = true;

        if (EpsilonEquals(desiredState.vx, 0.0) && EpsilonEquals(desiredState.vy, 0.0)
            && EpsilonEquals(desiredState.omega, 0.0))
        {
            needToRotate = false;
        }

        // For each module, compute local Vx and Vy vectors.
        struct ModuleData
        {
            double prevVX;
            double prevVY;
            frc::Rotation2d prevHeading;
            double desiredVX;
            double desiredVY;
            frc::Rotation2d desiredHeading;
        };

        std::array<ModuleData, 4> moduleData;

        bool all_modules_should_flip = true;

        for (int i = 0; i < 4; ++i)
        {
            moduleData[i]
                = { CosDegrees(previousSetpoint.moduleStates[i].angle) * previousSetpoint.moduleStates[i].velocity,
                    SinDegrees(previousSetpoint.moduleStates[i].angle) * previousSetpoint.moduleStates[i].velocity,
                    frc::Rotation2d{ units::degree_t{ previousSetpoint.moduleStates[i].angle } },
                    CosDegrees(desiredModuleStates[i].angle) * desiredModuleStates[i].velocity,
                    SinDegrees(desiredModuleStates[i].angle) * desiredModuleStates[i].velocity,
                    frc::Rotation2d{ units::degree_t{ desiredModuleStates[i].angle } } };

            if (previousSetpoint.moduleStates[i].velocity < 0.0)
            {
                moduleData[i].prevHeading = moduleData[i].prevHeading.RotateBy(180_deg);
            }

            if (desiredModuleStates[i].velocity < 0.0)
            {
                moduleData[i].desiredHeading = moduleData[i].desiredHeading.RotateBy(180_deg);
            }

            if (all_modules_should_flip)
            {
                double required_rotation_rad
                    = fabs(frc::Rotation2d{ units::degree_t{ -moduleData[i].prevHeading.Degrees().value() } }
                               .RotateBy(moduleData[i].desiredHeading)
                               .Degrees()
                               .value());
                if (required_rotation_rad < 90)
                {
                    all_modules_should_flip = false;
                }
            }
        }

        if (all_modules_should_flip
            && !(EpsilonEquals(previousSetpoint.chassisSpeeds.vx, 0.0)
                 && EpsilonEquals(previousSetpoint.chassisSpeeds.vy, 0.0)
                 && EpsilonEquals(previousSetpoint.chassisSpeeds.omega, 0.0))
            && !(EpsilonEquals(desiredState.vx, 0.0) && EpsilonEquals(desiredState.vy, 0.0)
                 && EpsilonEquals(desiredState.omega, 0.0)))
        {
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement of the desired
            // angle, and accelerate again.
            return GenerateSetpoint(limits, previousSetpoint, CowChassisSpeeds{ 0, 0, 0 }, dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
        double dx     = desiredState.vx - previousSetpoint.chassisSpeeds.vx;
        double dy     = desiredState.vy - previousSetpoint.chassisSpeeds.vy;
        double dtheta = desiredState.omega - previousSetpoint.chassisSpeeds.omega;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at desiredState.
        double minS = 1.0;

        std::array<std::optional<frc::Rotation2d>, 4> overrideRotation;

        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the minimum across all modules, since
        // that is the active constraint.
        double maxThetaStep = dt * limits.maxRotationalVelocity;
        for (int i = 0; i < 4; ++i)
        {
            if (!needToRotate)
            {
                overrideRotation[i]
                    = std::optional<frc::Rotation2d>{ units::degree_t{ previousSetpoint.moduleStates[i].angle } };
                //  (Optional.of(prevSetpoint.mModuleStates[i].angle));
                continue;
            }
            overrideRotation[i] = std::nullopt;

            if (EpsilonEquals(previousSetpoint.moduleStates[i].velocity, 0.0))
            {
                // If module is stopped, we know that we will need to move straight to the final steering angle, so limit based
                // purely on rotation in place.
                if (EpsilonEquals(desiredModuleStates[i].velocity, 0.0))
                {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideRotation[i]
                        = std::optional<frc::Rotation2d>{ units::degree_t{ previousSetpoint.moduleStates[i].angle } };
                    continue;
                }

                auto necessaryRotation
                    = frc::Rotation2d{ units::degree_t{ -previousSetpoint.moduleStates[i].angle } }.RotateBy(
                        frc::Rotation2d{ units::degree_t{ desiredModuleStates[i].angle } });
                if (FlipHeading(necessaryRotation))
                {
                    necessaryRotation = necessaryRotation.RotateBy(180_deg);
                }
                // getRadians() bounds to +/- Pi.
                double numStepsNeeded = fabs(necessaryRotation.Degrees().value()) / maxThetaStep;

                if (numStepsNeeded <= 1.0)
                {
                    // Steer directly to goal angle.
                    overrideRotation[i]
                        = std::optional<frc::Rotation2d>{ units::degree_t{ desiredModuleStates[i].angle } };
                    // Don't limit the global min_s;
                    continue;
                }
                else
                {
                    // Adjust steering by max_theta_step.
                    auto necessary = necessaryRotation.Degrees().value();
                    auto sgn       = (necessary > 0) ? 1 : ((necessary < 0) ? -1 : 0);
                    auto rotation
                        = frc::Rotation2d{ units::degree_t{ previousSetpoint.moduleStates[i].angle } }.RotateBy(
                            frc::Rotation2d{ units::degree_t{ sgn * maxThetaStep } });

                    overrideRotation[i] = std::optional<frc::Rotation2d>{ rotation };

                    minS = 0.0;
                    continue;
                }
            }

            if (minS == 0.0)
            {
                // s can't get any lower. Save some CPU.
                continue;
            }

            int kMaxIterations = 8;
            double s           = FindSteeringMaxS(moduleData[i].prevVX,
                                        moduleData[i].prevVY,
                                        moduleData[i].prevHeading.Radians().value(),
                                        moduleData[i].desiredVX,
                                        moduleData[i].desiredVY,
                                        moduleData[i].desiredHeading.Radians().value(),
                                        maxThetaStep,
                                        kMaxIterations);
            minS               = fmin(minS, s);
        }

        // Enforce drive wheel acceleration limits.
        double maxVelStep = dt * limits.maxDriveAcceleration;
        for (int i = 0; i < 4; ++i)
        {
            if (minS == 0.0)
            {
                // No need to carry on.
                break;
            }
            double vx_min_s = minS == 1.0
                                  ? moduleData[i].desiredVX
                                  : (moduleData[i].desiredVX - moduleData[i].prevVX) * minS + moduleData[i].prevVX;
            double vy_min_s = minS == 1.0
                                  ? moduleData[i].desiredVY
                                  : (moduleData[i].desiredVY - moduleData[i].prevVY) * minS + moduleData[i].prevVY;
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we already know we can't go faster
            // than that.
            // TODO(for efficiency, do all this on v^2 to save a bunch of sqrts)
            // TODO(be smarter about root finding, since this is just a quadratic in s: ((xf-x0)*s+x0)^2+((yf-y0)*s+y0)^2)
            int kMaxIterations = 10;
            double s           = minS
                       * FindDriveMaxS(moduleData[i].prevVX,
                                       moduleData[i].prevVY,
                                       std::hypot(moduleData[i].prevVX, moduleData[i].prevVY),
                                       vx_min_s,
                                       vy_min_s,
                                       std::hypot(vx_min_s, vy_min_s),
                                       maxVelStep,
                                       kMaxIterations);
            minS = fmin(minS, s);
        }

        auto returnSpeeds = CowChassisSpeeds{ previousSetpoint.chassisSpeeds.vx + minS * dx,
                                              previousSetpoint.chassisSpeeds.vy + minS * dy,
                                              previousSetpoint.chassisSpeeds.omega + minS * dtheta };

        auto returnStates = m_Kinematics->CalculateModuleStates(returnSpeeds, 0, 0);
                std::cout << "ret states 1: " << returnStates[0].angle << " " << returnStates[0].velocity
                  << " 2:" << returnStates[1].angle << " " << returnStates[1].velocity << " 3:" << returnStates[2].angle
                  << " " << returnStates[2].velocity << " 4:" << returnStates[3].angle << " "
                  << returnStates[3].velocity << std::endl;


        for (int i = 0; i < 4; ++i)
        {
            auto maybeOverride = overrideRotation[i];
            if (maybeOverride.has_value())
            {
                auto overrideVal = maybeOverride.value();
                if (FlipHeading(
                        frc::Rotation2d{ units::degree_t{ overrideVal.Degrees().value() - returnStates[i].angle } }))
                {
                    returnStates[i].velocity *= -1.0;
                }
                returnStates[i].angle = overrideVal.Degrees().value();
            }
            auto deltaRotation = returnStates[i].angle - previousSetpoint.moduleStates[i].angle;
            if (FlipHeading(frc::Rotation2d{ units::degree_t{ deltaRotation } }))
            {
                returnStates[i].angle = returnStates[i].angle + 180;
                returnStates[i].velocity *= -1.0;
            }
        }

        return SwerveSetpoint{ returnSpeeds, returnStates };
    }

    bool CowSwerveSetpointGenerator::FlipHeading(frc::Rotation2d previousToGoal)
    {
        return fabs(previousToGoal.Degrees().value()) > 90.0;
    }

    double CowSwerveSetpointGenerator::UnwrapAngle(double scopeReference, double newAngle)
    {
        double lowerBound;
        double upperBound;
        double lowerOffset = std::fmod(scopeReference, 360);

        if (lowerOffset >= 0)
        {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        }
        else
        {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)
        {
            newAngle += 360;
        }

        while (newAngle > upperBound)
        {
            newAngle -= 360;
        }

        if (newAngle - scopeReference > 180)
        {
            newAngle -= 360;
        }
        else if (newAngle - scopeReference < -180)
        {
            newAngle += 360;
        }

        return newAngle;
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique. This is a pretty naive way to
     * do root finding, but it's usually faster than simple bisection while being robust in ways that e.g. the Newton-Raphson
     * method isn't.
     * @param func The Function2d to take the root of.
     * @param x0 x value of the lower bracket.
     * @param y0 y value of the lower bracket.
     * @param f0 value of 'func' at x0, y_0 (passed in by caller to save a call to 'func' during recursion)
     * @param x1 x value of the upper bracket.
     * @param y1 y value of the upper bracket.
     * @param f1 value of 'func' at x1, y1 (passed in by caller to save a call to 'func' during recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the (approximate) root.
     */
    double CowSwerveSetpointGenerator::FindRoot(Function2d func,
                                                double x0,
                                                double y0,
                                                double f0,
                                                double x1,
                                                double y1,
                                                double f1,
                                                int iterationsLeft)
    {
        if (iterationsLeft < 0 || EpsilonEquals(f0, f1))
        {
            return 1.0;
        }
        auto s_guess = fmax(0.0, fmin(1.0, -f0 / (f1 - f0)));
        auto x_guess = (x1 - x0) * s_guess + x0;
        auto y_guess = (y1 - y0) * s_guess + y0;
        auto f_guess = func(x_guess, y_guess);
        if (sgn(f0) == sgn(f_guess))
        {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                   + (1.0 - s_guess) * FindRoot(func, x_guess, y_guess, f_guess, x1, y1, f1, iterationsLeft - 1);
        }
        else
        {
            // Use lower bracket.
            return s_guess * FindRoot(func, x0, y0, f0, x_guess, y_guess, f_guess, iterationsLeft - 1);
        }
    }

    double CowSwerveSetpointGenerator::FindSteeringMaxS(double x0,
                                                        double y0,
                                                        double f0,
                                                        double x1,
                                                        double y1,
                                                        double f1,
                                                        double maxDeviation,
                                                        int maxIterations)
    {
        f1          = UnwrapAngle(f0, f1);
        double diff = f1 - f0;
        if (fabs(diff) <= maxDeviation)
        {
            return 1.0;
        }

        double offset   = f0 + sgn(diff) * maxDeviation;
        Function2d func = [this, f0, offset](double x, double y) { return UnwrapAngle(f0, atan2(y, x)) - offset; };
        return FindRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    double CowSwerveSetpointGenerator::FindDriveMaxS(double x0,
                                                     double y0,
                                                     double f0,
                                                     double x1,
                                                     double y1,
                                                     double f1,
                                                     double maxVelStep,
                                                     int maxIterations)
    {
        double diff = f1 - f0;
        if (fabs(diff) <= maxVelStep)
        {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset   = f0 + sgn(diff) * maxVelStep;
        Function2d func = [offset](double x, double y) { return std::hypot(x, y) - offset; };
        return FindRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    double CowSwerveSetpointGenerator::SinDegrees(double angle)
    {
        return sin(angle * M_PI / 180.0);
    }

    double CowSwerveSetpointGenerator::CosDegrees(double angle)
    {
        return cos(angle * M_PI / 180.0);
    }

} // namespace CowLib