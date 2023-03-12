#ifndef __COWLIB_INTERNAL_SWERVE_KINEMATICS_H__
#define __COWLIB_INTERNAL_SWERVE_KINEMATICS_H__

#include "ExtendedWPISwerveModuleState.h"
#include "units/angular_velocity.h"

#include <algorithm>
#include <frc/EigenCore.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

namespace CowLib
{
    template<size_t NumModules> class InternalSwerveKinematics : public frc::SwerveDriveKinematics<4>
    {
    private:
        mutable frc::Matrixd<NumModules * 2, 3> m_InverseKinematics;
        Eigen::HouseholderQR<frc::Matrixd<NumModules * 2, 3>> m_ForwardKinematics;

        mutable frc::Matrixd<NumModules * 2, 4> m_SecondOrderKinematics;

        std::array<frc::Translation2d, NumModules> m_Modules;
        mutable std::array<CowLib::ExtendedWPISwerveModuleState, NumModules> m_ModuleStates;

        mutable frc::Translation2d m_PreviousCoR;

    public:
        template<typename... Wheels>
        InternalSwerveKinematics(frc::Translation2d wheel, Wheels &&...wheels)
            : frc::SwerveDriveKinematics<NumModules>(wheel, wheels...)
        {
            m_Modules = { wheel, wheels... };

            static_assert(sizeof...(wheels) >= 1, "A swerve drive requires at least two modules");

            for (size_t i = 0; i < NumModules; i++)
            {
                // clang-format off
                m_InverseKinematics.template block<2, 3>(i * 2, 0) <<
                    1, 0, (-m_Modules[i].Y()).value(),
                    0, 1, (+m_Modules[i].X()).value();

                m_SecondOrderKinematics.template block <2, 4>(i * 2, 0) <<
                    1, 0, (-m_Modules[i].X()).value(), (-m_Modules[i].Y()).value(),
                    0, 1, (-m_Modules[i].Y()).value(), (m_Modules[i].X()).value();
                // clang-format on
            }

            m_ForwardKinematics = m_InverseKinematics.householderQr();

            wpi::math::MathSharedStore::ReportUsage(wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
        }

        // This constructor gives linker errors
        // InternalSwerveKinematics(const wpi::array<frc::Translation2d, 4> &wheels)
        //     : frc::SwerveDriveKinematics<4>(wheels)
        // {
        //     m_Modules = wheels;

        //     for (size_t i = 0; i < NumModules; i++)
        //     {
        //         // clang-format off
        //         m_InverseKinematics.template block<2, 3>(i * 2, 0) <<
        //             1, 0, (-m_Modules[i].Y()).value(),
        //             0, 1, (+m_Modules[i].X()).value();

        //         m_SecondOrderKinematics.template block <2, 4>(i * 2, 0) <<
        //             1, 0, (-m_Modules[i].X()).value(), (-m_Modules[i].Y()).value(),
        //             0, 1, (-m_Modules[i].Y()).value(), (m_Modules[i].X()).value();
        //         // clang-format on
        //     }

        //     m_ForwardKinematics = m_InverseKinematics.householderQr();

        //     wpi::math::MathSharedStore::ReportUsage(wpi::math::MathUsageId::kKinematics_SwerveDrive, 1);
        // }

        std::array<CowLib::ExtendedWPISwerveModuleState, NumModules>
        ToSwerveModuleStates(const frc::ChassisSpeeds &chassisSpeeds,
                             const frc::Translation2d &centerOfRotation = frc::Translation2d{}) const
        {
            if (chassisSpeeds.vx == 0_mps && chassisSpeeds.vy == 0_mps && chassisSpeeds.omega == 0_rad_per_s)
            {
                for (size_t i = 0; i < NumModules; i++)
                {
                    m_ModuleStates[i].speed = 0_mps;
                }

                return m_ModuleStates;
            }

            if (centerOfRotation != m_PreviousCoR)
            {
                for (size_t i = 0; i < NumModules; i++)
                {
                    // clang-format off
                    m_InverseKinematics.template block<2, 3>(i * 2, 0) <<
                        1, 0, 0, (-m_Modules[i].Y() + centerOfRotation.Y()).value(),
                        0, 1, 0, (m_Modules[i].X() - centerOfRotation.X()).value();

                    m_SecondOrderKinematics.template block<2, 4>(i * 2, 0) <<
                        1, 0, (-m_Modules[i].X() + centerOfRotation.X()).value(), (-m_Modules[i].Y() + centerOfRotation.Y()).value(),
                        0, 1, (-m_Modules[i].Y() + centerOfRotation.Y()).value(), (m_Modules[i].X() - centerOfRotation.X()).value();
                    // clang-format on
                }

                m_PreviousCoR = centerOfRotation;
            }

            Eigen::Vector3<double> chassisSpeedsVector{ chassisSpeeds.vx.value(),
                                                        chassisSpeeds.vy.value(),
                                                        chassisSpeeds.omega.value() };

            auto moduleVelocityStatesMatrix = m_InverseKinematics * chassisSpeedsVector;

            Eigen::Vector4<double> accelerationVector{ 0, 0, pow(chassisSpeeds.omega.value(), 2), 0 };

            auto moduleAccelerationStatesMatrix = m_SecondOrderKinematics * accelerationVector;

            for (size_t i = 0; i < NumModules; i++)
            {
                double x = moduleVelocityStatesMatrix(i * 2, 0);
                double y = moduleVelocityStatesMatrix(i * 2 + 1, 0);

                double ax = moduleAccelerationStatesMatrix(i * 2, 0);
                double ay = moduleAccelerationStatesMatrix(i * 2 + 1, 0);

                double speed = std::hypot(x, y);
                auto angle   = frc::Rotation2d(x, y);

                frc::Matrixd<2, 2> trigThetaAngle;
                trigThetaAngle.template block<2, 2>(0, 0) << angle.Cos(), -angle.Sin(), angle.Sin(), angle.Cos();

                Eigen::Vector2<double> accelVector{ ax, ay };

                auto omegaVector = trigThetaAngle * accelVector;

                double omega      = (omegaVector(1, 0) / speed) - chassisSpeeds.omega.value();
                m_ModuleStates[i] = ExtendedWPISwerveModuleState(units::meters_per_second_t{ speed },
                                                                 angle,
                                                                 units::radians_per_second_t{ omega });
            }

            return m_ModuleStates;
        }

        template<typename... ModuleStates> frc::ChassisSpeeds ToChassisSpeeds(ModuleStates &&...wheelStates) const
        {
            static_assert(sizeof...(wheelStates) == NumModules,
                          "Number of modules is not consistent with number of wheel "
                          "locations provided in constructor.");

            std::array<ExtendedWPISwerveModuleState, NumModules> moduleStates{ wheelStates... };

            return this->ToChassisSpeeds(moduleStates);
        }

        frc::ChassisSpeeds
        ToChassisSpeeds(std::array<CowLib::ExtendedWPISwerveModuleState, NumModules> moduleStates) const
        {
            frc::Matrixd<NumModules * 2, 1> moduleStatesMatrix;

            for (size_t i = 0; i < NumModules; i++)
            {
                moduleStatesMatrix(i * 2, 0)     = moduleStates[i].speed.value() * moduleStates[i].angle.Cos();
                moduleStatesMatrix(i * 2 + 1, 0) = moduleStates[i].speed.value() * moduleStates[i].angle.Sin();
            }

            auto chassisSpeedsVector = m_ForwardKinematics.solve(moduleStatesMatrix);

            return frc::ChassisSpeeds{ units::meters_per_second_t{ chassisSpeedsVector(0, 0) },
                                       units::meters_per_second_t{ chassisSpeedsVector(1, 0) },
                                       units::radians_per_second_t{ chassisSpeedsVector(2, 0) } };
        }

        template<typename... ModuleDeltas> frc::Twist2d ToTwist2d(ModuleDeltas &&...wheelDeltas) const
        {
            static_assert(sizeof...(wheelDeltas) == NumModules,
                          "Number of modules is not consistent with number of wheel "
                          "locations provided in constructor.");

            std::array<ExtendedWPISwerveModuleState, NumModules> moduleDeltas{ wheelDeltas... };

            return this->ToTwist2d(moduleDeltas);
        }

        frc::Twist2d ToTwist2d(std::array<frc::SwerveModulePosition, NumModules> wheelDeltas) const
        {
            frc::Matrixd<NumModules * 2, 1> moduleDeltaMatrix;

            for (size_t i = 0; i < NumModules; ++i)
            {
                frc::SwerveModulePosition module = wheelDeltas[i];
                moduleDeltaMatrix(i * 2, 0)      = module.distance.value() * module.angle.Cos();
                moduleDeltaMatrix(i * 2 + 1, 0)  = module.distance.value() * module.angle.Sin();
            }

            Eigen::Vector3d chassisDeltaVector = m_ForwardKinematics.solve(moduleDeltaMatrix);

            return { units::meter_t{ chassisDeltaVector(0) },
                     units::meter_t{ chassisDeltaVector(1) },
                     units::radian_t{ chassisDeltaVector(2) } };
        }

        static void DesaturateWheelSpeeds(std::array<CowLib::ExtendedWPISwerveModuleState, NumModules> *moduleStates,
                                          units::meters_per_second_t attainableMaxSpeed)
        {
            auto &states      = *moduleStates;
            auto realMaxSpeed = std::max_element(states.begin(),
                                                 states.end(),
                                                 [](const auto &a, const auto &b)
                                                 { return units::math::abs(a.speed) < units::math::abs(b.speed); })
                                    ->speed;

            if (realMaxSpeed > attainableMaxSpeed)
            {
                for (auto &module : states)
                {
                    module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
                }
            }
        }

        static void DesaturateWheelSpeeds(std::array<CowLib::ExtendedWPISwerveModuleState, NumModules> *moduleStates,
                                          frc::ChassisSpeeds currentChassisSpeed,
                                          units::meters_per_second_t attainableMaxModuleSpeed,
                                          units::meters_per_second_t attainableMaxRobotTranslationSpeed,
                                          units::radians_per_second_t attainableMaxRobotRotationSpeed)
        {
            auto &states = *moduleStates;

            auto realMaxSpeed = std::max_element(states.begin(),
                                                 states.end(),
                                                 [](const auto &a, const auto &b)
                                                 { return units::math::abs(a.speed) < units::math::abs(b.speed); })
                                    ->speed;

            if (attainableMaxRobotTranslationSpeed == 0_mps || attainableMaxRobotRotationSpeed == 0_rad_per_s
                || realMaxSpeed == 0_mps)
            {
                return;
            }

            auto translationalK = units::math::hypot(currentChassisSpeed.vx, currentChassisSpeed.vy)
                                  / attainableMaxRobotTranslationSpeed;

            auto rotationalK = units::math::abs(currentChassisSpeed.omega) / attainableMaxRobotRotationSpeed;

            auto k = units::math::max(translationalK, rotationalK);

            auto scale = units::math::min(k * attainableMaxModuleSpeed / realMaxSpeed, units::scalar_t(1));
            for (auto &module : states)
            {
                module.speed = module.speed * scale;
            }
        }
    };
} // namespace CowLib

#endif /* __COWLIB_INTERNAL_SWERVE_KINEMATICS_H__ */