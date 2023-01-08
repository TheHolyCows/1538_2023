#include "CowSwerveKinematics.h"

namespace CowLib {

/**
 * @brief Creates a new CowSwerveKinematics object
 * @param wheelBase distance between the center of each swerve module on one edge
 */
CowSwerveKinematics::CowSwerveKinematics(double wheelBase)
{
    auto p = units::foot_t { wheelBase / 2.0 };
    auto n = units::foot_t { wheelBase / -2.0 };

    m_Kinematics = new frc::SwerveDriveKinematics<4>(
        frc::Translation2d { p, p },
        frc::Translation2d { p, n },
        frc::Translation2d { n, p },
        frc::Translation2d { n, n });
}

CowSwerveKinematics::~CowSwerveKinematics()
{
    delete m_Kinematics;
}

/**
 * @brief Re-normalizes wheel speeds if any individual speed is above maxSpeed
 * @param moduleStates Reference to std::array of CowSwerveModuleStates (mutated)
 * @param maxSpeed Maximum attainable speed (feet per second)
 */
void CowSwerveKinematics::DesaturateSpeeds(std::array<CowSwerveModuleState, 4>* moduleStates, double maxSpeed)
{
    auto& states = *moduleStates;

    auto highestModuleSpeed = std::max_element(states.begin(), states.end(), [](const auto& a, const auto& b) {
        return fabs(a.velocity) < fabs(b.velocity);
    })->velocity;

    if (highestModuleSpeed > maxSpeed) {
        for (auto& module : states) {
            module.velocity = module.velocity / highestModuleSpeed * maxSpeed;
        }
    }
}

/**
 * @brief Uses WPILib kinematics to calculate module states from chassisSpeeds
 * @param chassisSpeeds CowChassisSpeeds struct
 * @return std::array of 4 CowSwerveModuleStates
 */
std::array<CowSwerveModuleState, 4> CowSwerveKinematics::CalculateModuleStates(CowChassisSpeeds& chassisSpeeds)
{
    auto moduleStates = m_Kinematics->ToSwerveModuleStates(chassisSpeeds.ToWPI());

    std::array<CowSwerveModuleState, 4> convertedStates {};

    for (int i = 0; i < 4; i++) {
        double velocity = moduleStates[i].speed.convert<units::feet_per_second>().value();
        double angle = moduleStates[i].angle.Degrees().value();

        convertedStates[i] = CowSwerveModuleState { velocity, angle };
    }

    return convertedStates;
}

/**
 * @brief Retrieves the internal kinematics instance
 * @return WPILib SwerveDriveKinematics
 */
frc::SwerveDriveKinematics<4>* CowSwerveKinematics::GetInternalKinematics()
{
    return m_Kinematics;
}

}