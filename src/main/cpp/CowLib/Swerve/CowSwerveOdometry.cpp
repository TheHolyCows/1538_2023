#include "./CowSwerveOdometry.h"

namespace CowLib {

/**
 * @brief Creates a new CowSwerveOdometry instance
 * @param kinematics Pointer to CowSwerveKinematics instance
 * @param gyroAngle current gyro angle
 * @param initialX starting X translation in feet
 * @param initialY starting Y translation in feet
 * @param initialRotation starting rotation in degrees
 */
CowSwerveOdometry::CowSwerveOdometry(CowSwerveKinematics* kinematics, double gyroAngle, double initialX, double initialY, double initialRotation)
{
    wpi::array<frc::SwerveModulePosition,4> pos = {frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg)};
    m_Odometry = new frc::SwerveDriveOdometry<4>(
        *(kinematics->GetInternalKinematics()),
        frc::Rotation2d(units::degree_t { gyroAngle }),
        pos,
        CreateWPIPose(initialX, initialY, initialRotation));
}

CowSwerveOdometry::~CowSwerveOdometry()
{
    delete m_Odometry;
}

/**
 * Converts 3 doubles to WPILib Pose2d
 * @param x x position in feet
 * @param y y position in feet
 * @param rotation robot rotation in degrees
 * @return generated Pose2d
 */
frc::Pose2d CowSwerveOdometry::CreateWPIPose(double x, double y, double rotation)
{
    return frc::Pose2d(units::foot_t { x }, units::foot_t { y }, frc::Rotation2d(units::degree_t { rotation }));
}

/**
 * @brief Converts an array of CowSwerveModuleStates to WPILib SwerveModuleStates. Utalized ToWPI method on CowSwerveModuleState struct.
 * @param moduleStates std::array of 4 CowSwerveModuleStates
 * @return std::array of 4 WPILib SwerveModuleStates
 */
std::array<frc::SwerveModuleState, 4> CowSwerveOdometry::CreateWPIModuleStates(std::array<CowSwerveModuleState, 4> moduleStates)
{
    std::array<frc::SwerveModuleState, 4> WPIModuleStates;

    // Converts each CowSwerveModuleState to a WPILib module state and places in the new std::array
    std::transform(moduleStates.begin(), moduleStates.end(), WPIModuleStates.begin(), [](CowSwerveModuleState state) { return state.ToWPI(); });

    return WPIModuleStates;
}

/**
 * @brief Resets the robot position on the field.
 * @param newX new X position in feet
 * @param newY new Y position in feet
 * @param newRotation new robot rotation in degrees
 * @param gyroAngle current gyro angle in degrees
 */
void CowSwerveOdometry::Reset(double newX, double newY, double newRotation, double gyroAngle)
{
    wpi::array<frc::SwerveModulePosition,4> pos = {frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg),frc::SwerveModulePosition(0_m,0_deg)};
    m_Odometry->ResetPosition(frc::Rotation2d(units::degree_t { gyroAngle }),pos, CreateWPIPose(newX, newY, newRotation));
    m_Pose = m_Odometry->GetPose();
}

/**
 * @brief Gets current X position of the robot
 * @return X position in feet
 */
double CowSwerveOdometry::GetX()
{
    return m_Pose.X().convert<units::foot>().value();
}

/**
 * @brief Gets current Y position of the robot
 * @return Y position in feet
 */
double CowSwerveOdometry::GetY()
{
    return m_Pose.Y().convert<units::foot>().value();
}

/**
 * @brief Gets current robot rotation in degrees
 * @return rotation in degrees
 */
double CowSwerveOdometry::GetRotation()
{
    return m_Pose.Rotation().Degrees().value();
}

/**
 * @brief Gets a WPILib Pose2d object of the robot's position
 * @return WPILib Pose2d
 */
frc::Pose2d CowSwerveOdometry::GetWPIPose()
{
    return m_Pose;
}

/**
 * @brief Updates the robot's position on the field using forward kinematics and integration of the pose over time
 * @param gyroAngle The angle reported by the gyroscope (in degrees)
 * @param moduleStates std::array of all 4 swerve module states. Must be in the same order.
 */
// void CowSwerveOdometry::Update(double gyroAngle, std::array<CowSwerveModuleState, 4> moduleStates)
// {
//     auto WPIModuleStates = CreateWPIModuleStates(moduleStates);

//     m_Pose = m_Odometry->Update(frc::Rotation2d(units::degree_t { gyroAngle }), WPIModuleStates[0], WPIModuleStates[1], WPIModuleStates[2], WPIModuleStates[3]);
// }

/**
 * @brief Updates the robot's position on the field using forward kinematics and integration of the pose over time.
 * @param currentTime The current time in seconds
 * @param gyroAngle The angle reported by the gyroscope (in degrees)
 * @param moduleStates std::array of all 4 swerve module states. Must be in the same order.
 */
// void CowSwerveOdometry::UpdateWithTime(double currentTime, double gyroAngle, std::array<CowSwerveModuleState, 4> moduleStates)
// {
//     auto WPIModuleStates = CreateWPIModuleStates(moduleStates);

//     m_Pose = m_Odometry->Update(frc::Rotation2d(units::degree_t { gyroAngle }),
//         WPIModuleStates[0], WPIModuleStates[1], WPIModuleStates[2], WPIModuleStates[3]);
// }

/**
 * @brief Retrieves the internal odometry instance
 * @return Pointer to WPILib SwerveDriveOdometry
 */
frc::SwerveDriveOdometry<4>* CowSwerveOdometry::GetInternalOdometry()
{
    return m_Odometry;
}

}