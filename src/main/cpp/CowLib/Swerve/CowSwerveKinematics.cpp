#include "CowSwerveKinematics.h"

#include <array>

namespace CowLib
{

    /**
 * @brief Creates a new CowSwerveKinematics object
 * @param wheelBase distance between the center of each swerve module on one edge
 */
    CowSwerveKinematics::CowSwerveKinematics(double wheelBase)
    {
        auto p = wheelBase / 2.0;
        auto n = wheelBase / -2.0;

        m_ModulePositions
            = { Translation2d{ p, p }, Translation2d{ p, n }, Translation2d{ n, p }, Translation2d{ n, n } };

        for (int i = 0; i < NUM_MODULES; i++)
        {
            // clang-format off
            m_InverseKinematics.template block<2, 3>(i * 2, 0) <<
                1, 0, -m_ModulePositions[i].Y(),
                0, 1, +m_ModulePositions[i].X();
            // clang-format on
        }

        m_ForwardKinematics = m_InverseKinematics.householderQr();

        m_ModuleStates
            = { CowSwerveModuleState(), CowSwerveModuleState(), CowSwerveModuleState(), CowSwerveModuleState() };
        m_PreviousCenterOfRotation = Translation2d();
    }

    CowSwerveKinematics::~CowSwerveKinematics()
    {
    }

    /**
 * @brief Re-normalizes wheel speeds if any individual speed is above maxSpeed
 * @param moduleStates Reference to std::array of CowSwerveModuleStates (mutated)
 * @param maxSpeed Maximum attainable speed (feet per second)
 */
    void CowSwerveKinematics::DesaturateSpeeds(std::array<CowSwerveModuleState, 4> *moduleStates, double maxSpeed)
    {
        // auto &states = *moduleStates;

        // auto highestModuleSpeed
        //     = std::max_element(states.begin(),
        //                        states.end(),
        //                        [](const auto &a, const auto &b) { return fabs(a.velocity) < fabs(b.velocity); })
        //           ->velocity;

        // if (highestModuleSpeed > maxSpeed)
        // {
        //     for (auto &module : states)
        //     {
        //         module.velocity = module.velocity / highestModuleSpeed * maxSpeed;
        //     }
        // }
    }

    /**
 * @brief Uses WPILib kinematics to calculate module states from chassisSpeeds
 * @param chassisSpeeds CowChassisSpeeds struct
 * @return std::array of 4 CowSwerveModuleStates
 */
    std::array<CowSwerveModuleState, 4>
    CowSwerveKinematics::CalculateModuleStates(const CowChassisSpeeds &chassisSpeeds,
                                               const Translation2d centerOfRotation) const
    {
        if (chassisSpeeds.vx == 0 && chassisSpeeds.vy == 0 && chassisSpeeds.omega == 0)
        {
            for (int i = 0; i < NUM_MODULES; i++)
            {
                m_ModuleStates[i].velocity = 0;
            }

            return m_ModuleStates;
        }

        // // We have a new center of rotation. We need to compute the matrix again.
        // if (m_PreviousCenterOfRotation != centerOfRotation)
        // {
        //     for (int i = 0; i < NUM_MODULES; i++)
        //     {
        //         // clang-format off
        //         m_InverseKinematics.template block<2, 3>(i * 2, 0) =
        //             frc::Matrixd<2, 3>{
        //             {1, 0, (-m_ModulePositions[i].Y() + centerOfRotation.Y())},
        //             {0, 1, (+m_ModulePositions[i].X() - centerOfRotation.X())}};
        //         // clang-format on
        //     }
        //     m_PreviousCenterOfRotation = centerOfRotation;
        // }

        // Eigen::Vector3d chassisSpeedsVector{ chassisSpeeds.vx.value(),
        //                                      chassisSpeeds.vy.value(),
        //                                      chassisSpeeds.omega.value() };

        // Matrixd<NumModules * 2, 1> moduleStateMatrix = m_inverseKinematics * chassisSpeedsVector;

        // for (size_t i = 0; i < NumModules; i++)
        // {
        //     units::meters_per_second_t x{ moduleStateMatrix(i * 2, 0) };
        //     units::meters_per_second_t y{ moduleStateMatrix(i * 2 + 1, 0) };

        //     auto speed = units::math::hypot(x, y);
        //     Rotation2d rotation{ x.value(), y.value() };

        //     m_moduleStates[i] = { speed, rotation };
        // }

        // return m_moduleStates;

        // frc::Translation2d centerOfRotation
        //     = frc::Translation2d(units::foot_t{ centerOfRotationX }, units::foot_t{ centerOfRotationY });

        // auto moduleStates = m_Kinematics->ToSwerveModuleStates(chassisSpeeds.ToWPI(), centerOfRotation);

        // std::array<CowSwerveModuleState, 4> convertedStates{};

        // for (int i = 0; i < 4; i++)
        // {
        //     double velocity = moduleStates[i].speed.convert<units::feet_per_second>().value();
        //     double angle    = moduleStates[i].angle.Degrees().value();

        //     convertedStates[i] = CowSwerveModuleState{ velocity, angle };
        // }

        // return convertedStates;
    }

    CowChassisSpeeds CowSwerveKinematics::CalculateChassisSpeeds(std::array<CowSwerveModuleState, 4> &moduleStates)
    {
        // return CowChassisSpeeds::FromWPI(m_Kinematics->ToChassisSpeeds(moduleStates[0].ToWPI(),
        //                                                                moduleStates[1].ToWPI(),
        //                                                                moduleStates[2].ToWPI(),
        //                                                                moduleStates[3].ToWPI()));
    }

    std::array<frc::Translation2d, 4> CowSwerveKinematics::GetModulePositions()
    {
        // return m_ModulePositions;
    }

    /**
 * @brief Retrieves the internal kinematics instance
 * @return WPILib SwerveDriveKinematics
 */
    frc::SwerveDriveKinematics<4> *CowSwerveKinematics::GetInternalKinematics()
    {
        // return m_Kinematics;
    }

} // namespace CowLib