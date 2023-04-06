#include "./CowSwerveOdometry.h"

#include "frc/smartdashboard/SmartDashboard.h"

namespace CowLib
{

    /**
     * @brief Creates a new CowSwerveOdometry instance
     * @param kinematics Pointer to CowSwerveKinematics instance
     * @param gyroAngle current gyro angle
     * @param initialX starting X translation in feet
     * @param initialY starting Y translation in feet
     * @param initialRotation starting rotation in degrees
     */
    CowSwerveOdometry::CowSwerveOdometry(CowSwerveKinematics *kinematics,
                                         double gyroAngle,
                                         double initialX,
                                         double initialY,
                                         double initialRotation)
    {
        // frc::SmartDashboard::PutData("field", &m_Field);

        std::array<frc::SwerveModulePosition, 4> zeroPositions;
        for (int i = 0; i < 4; i++)
        {
            zeroPositions[i] = frc::SwerveModulePosition{ 0_m, 0_deg };
        }

        m_Kinematics = kinematics;

        m_PoseEstimator = new frc::SwerveDrivePoseEstimator<4>(*(kinematics->GetInternalKinematics()),
                                                               frc::Rotation2d(units::degree_t{ gyroAngle }),
                                                               zeroPositions,
                                                               CreateWPIPose(initialX, initialY, initialRotation));
    }

    CowSwerveOdometry::~CowSwerveOdometry()
    {
        delete m_PoseEstimator;
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
        return frc::Pose2d(units::foot_t{ x }, units::foot_t{ y }, frc::Rotation2d(units::degree_t{ rotation }));
    }

    /**
     * @brief Converts an array of CowSwerveModulePositions to WPILib SwerveModulePositions.
     * Utilized ToWPI method on CowSwerveModulePosition struct.
     * @param modulePositions std::array of 4 CowSwerveModulePositions
     * @return std::array of 4 WPILib SwerveModulePositions
     */
    std::array<frc::SwerveModulePosition, 4>
    CowSwerveOdometry::CreateWPIModulePositions(std::array<CowSwerveModulePosition, 4> modulePositions)
    {
        std::array<frc::SwerveModulePosition, 4> WPIModulePositions;

        // Converts each CowSwerveModulePosition to a WPILib module position and places in the new std::array
        std::transform(modulePositions.begin(),
                       modulePositions.end(),
                       WPIModulePositions.begin(),
                       [](CowSwerveModulePosition position) { return position.ToWPI(); });

        return WPIModulePositions;
    }

    /**
     * @brief Resets the robot position on the field.
     * @param newX new X position in feet
     * @param newY new Y position in feet
     * @param newRotation new robot rotation in degrees
     * @param gyroAngle current gyro angle in degrees
     */
    void CowSwerveOdometry::Reset(double newX,
                                  double newY,
                                  double newRotation,
                                  double gyroAngle,
                                  std::array<CowLib::CowSwerveModulePosition, 4> modPositions)
    {
        Reset(CreateWPIPose(newX, newY, newRotation), gyroAngle, modPositions);
    }

    void CowSwerveOdometry::Reset(frc::Pose2d pose,
                                  double gyroAngle,
                                  std::array<CowLib::CowSwerveModulePosition, 4> modPositions)
    {
        m_PoseEstimator->ResetPosition(frc::Rotation2d(units::degree_t{ gyroAngle }),
                                       CreateWPIModulePositions(modPositions),
                                       pose);
        // delete m_PoseEstimator;
        // m_PoseEstimator = new frc::SwerveDrivePoseEstimator<4>(*(m_Kinematics->GetInternalKinematics()),
        //                                                        frc::Rotation2d(units::degree_t{ gyroAngle }),
        //                                                        CreateWPIModulePositions(modPositions),
        //                                                        pose);

        m_Pose = m_PoseEstimator->GetEstimatedPosition();
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
     * @param modulePositions std::array of all 4 swerve module positions. Must be in the same order.
     */
    void CowSwerveOdometry::Update(double gyroAngle, std::array<CowSwerveModulePosition, 4> modulePositions)
    {
        std::array<frc::SwerveModulePosition, 4> WPIModulePositions = CreateWPIModulePositions(modulePositions);

        m_Pose = m_PoseEstimator->Update(frc::Rotation2d(units::degree_t{ gyroAngle }), WPIModulePositions);

        m_Field.SetRobotPose(m_Pose);
    }

    /**
     * @brief Retrieves the internal pose estimator instance
     * @return Pointer to WPILib SwerveDrivePoseEstimator
     */
    frc::SwerveDrivePoseEstimator<4> *CowSwerveOdometry::GetInternalPoseEstimator()
    {
        return m_PoseEstimator;
    }

} // namespace CowLib
