#include "SwerveDrive.h"

#include "frc/RobotBase.h"
#include "SwerveModuleSim.h"

/**
 * @brief Construct a new SwerveDrive object
 *
 * @param moduleConstants Array of constants for each module
 */
SwerveDrive::SwerveDrive(ModuleConstants moduleConstants[4], double wheelBase)
{
    m_Gyro = CowPigeon::GetInstance();

    m_Locked = false;

    if (!frc::RobotBase::IsReal())
    {
        for (int i = 0; i < 4; i++)
        {
            m_Modules[i] = new SwerveModuleSim(i, 8, 500, 0, moduleConstants[i].encoderOffset);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            m_Modules[i] = new SwerveModule(i,
                                            moduleConstants[i].driveMotorId,
                                            moduleConstants[i].rotationMotorId,
                                            moduleConstants[i].encoderId,
                                            moduleConstants[i].encoderOffset);
        }
    }

    m_Kinematics = new CowLib::CowSwerveKinematics(wheelBase);

    m_Odometry = new CowLib::CowSwerveOdometry(m_Kinematics, m_Gyro->GetYawDegrees(), 0, 0, 0);

    // m_VisionPIDController = new CowLib::CowPID(CONSTANT("SWERVE_VISION_P"), CONSTANT("SWERVE_VISION_I"),
    // CONSTANT("SWERVE_VISION_D"), 0);

    // m_VisionPIDController->SetContinuous(true);
    // m_VisionPIDController->SetInputRange(-180, 180);

    Reset();
}

SwerveDrive::~SwerveDrive()
{
    delete m_Odometry;
    delete m_Kinematics;

    for (auto module : m_Modules)
    {
        delete module;
    }

    // delete m_VisionPIDController;
}

/**
 * @brief Sets drive velocity
 *
 * @param x Translational X velocity in feet per second
 * @param y Translational Y velocity in feet per second
 * @param rotation Rotational velocity in degrees per second
 * @param isFieldRelative Controls whether drive is field relative, default true
 * @param centerOfRotationX X component of center of rotation
 * @param centerOfRotationY Y component of center of rotation
 */
void SwerveDrive::SetVelocity(double vx,
                              double vy,
                              double omega,
                              bool isFieldRelative,
                              double centerOfRotationX,
                              double centerOfRotationY,
                              bool force)
{
    CowLib::CowChassisSpeeds chassisSpeeds{};

    // units::feet_per_second_t vx { x };
    // units::feet_per_second_t vy { y };
    // units::degrees_per_second_t rotationalVelocity { rotation };

    // frc::SmartDashboard::PutNumber("Gyro angle", m_Gyro->GetAngle());

    if (isFieldRelative)
    {
        // How does this know what angle it starts at
        chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, m_Gyro->GetYawDegrees());
    }
    else
    {
        chassisSpeeds = CowLib::CowChassisSpeeds{ vx, vy, omega };
    }

    std::array<CowLib::CowSwerveModuleState, 4> moduleStates;

    if (m_Locked)
    {
        // hardcoded angles for e-brake X formation
        double angles[4] = { 45, 315, 135, 225 };

        for (int i = 0; i < 4; i++)
        {
            moduleStates[i] = CowLib::CowSwerveModuleState{ 0.0, angles[i] };
        }
    }
    else
    {
        moduleStates = m_Kinematics->CalculateModuleStates(chassisSpeeds, centerOfRotationX, centerOfRotationY);
    }

    // Just in case
    CowLib::CowSwerveKinematics::DesaturateSpeeds(&moduleStates, CONSTANT("SWERVE_MAX_SPEED"));

    for (auto module : m_Modules)
    {
        module->SetTargetState(moduleStates[module->GetID()], m_Locked);
    }
}

/**
 * @brief Same as the other SetVelocity, but using CowChassisSpeeds
 * @param chassisSpeeds CowChassisSpeeds struct
 * @param isFieldRelative
 * @param centerOfRotationX X component of center of rotation
 * @param centerOfRotationY Y component of center of rotation
 */
void SwerveDrive::SetVelocity(CowLib::CowChassisSpeeds chassisSpeeds,
                              bool isFieldRelative,
                              double centerOfRotationX,
                              double centerOfRotationY,
                              bool force)
{
    SetVelocity(chassisSpeeds.vx,
                chassisSpeeds.vy,
                chassisSpeeds.omega,
                isFieldRelative,
                centerOfRotationX,
                centerOfRotationY,
                force);
}

/**
 * @brief Get the Pose X value in feet
 * 
 * @return double 
 */
double SwerveDrive::GetPoseX()
{
    return m_Pose.X().convert<units::foot>().value();
}

/**
 * @brief Get the Pose Y value in feet
 * 
 * @return double 
 */
double SwerveDrive::GetPoseY()
{
    return m_Pose.Y().convert<units::foot>().value();
}

/**
 * @brief Get the pose rotation value in degrees
 * 
 * @return double 
 */
double SwerveDrive::GetPoseRot()
{
    return m_Pose.Rotation().Degrees().value();
}

/**
 * @brief Returns current locked state
 *
 * @return Whether drive is locked
 */
bool SwerveDrive::GetLocked() const
{
    return m_Locked;
}

/**
 * @brief Sets locked state
 * moves wheels into X formation, braking in place on charge station
 * call to this within Operator Controller is all that is needed to e-brake
 * @param isLocked
 */
void SwerveDrive::SetLocked(bool isLocked)
{
    m_Locked = isLocked;
}

void SwerveDrive::ResetConstants()
{
    for (auto module : m_Modules)
    {
        module->ResetConstants();
    }
}

/// @brief Resets encoders
void SwerveDrive::ResetEncoders()
{
    for (auto module : m_Modules)
    {
        module->ResetEncoders();
    }
}

void SwerveDrive::ResetOdometry(frc::Pose2d pose)
{
    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions{};
    for (auto module : m_Modules)
    {
        modulePositions[module->GetID()] = module->GetPosition();
    }

    m_Odometry->Reset(pose, pose.Rotation().Degrees().value(), modulePositions);
    m_Gyro->SetYaw(pose.Rotation().Degrees().value());
}

void SwerveDrive::AddVisionMeasurement(frc::Pose2d pose, double timestamp)
{
    m_Odometry->GetInternalPoseEstimator()->AddVisionMeasurement(pose, units::second_t{ timestamp });
}

void SwerveDrive::Handle()
{
    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions{};
    // std::array<CowLib::CowSwerveModuleState, 4> moduleStates{};
    for (auto module : m_Modules)
    {
        module->Handle();
        modulePositions[module->GetID()] = module->GetPosition();
        // moduleStates[module->GetID()]    = module->GetState();
    }

    // if (!frc::RobotBase::IsReal())
    // {
    //     m_Gyro->SetYaw(m_Kinematics->GetInternalKinematics()
    //                        ->ToChassisSpeeds(CowLib::CowSwerveModuleState::ToWPIExtendedArray(moduleStates))
    //                        .omega.value()
    //                    + m_Gyro->GetYawDegrees());
    // }

    m_Odometry->Update(m_Gyro->GetYawDegrees(), modulePositions);

    m_Pose = m_Odometry->GetWPIPose();
}

void SwerveDrive::SetBrakeMode(bool brakeMode)
{
    for (auto module : m_Modules)
    {
        module->SetBrakeMode(brakeMode);
    }
}
