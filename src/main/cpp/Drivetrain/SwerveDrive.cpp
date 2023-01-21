#include "SwerveDrive.h"

/**
 * @brief Construct a new SwerveDrive object
 *
 * @param moduleConstants Array of constants for each module
 */
SwerveDrive::SwerveDrive(ModuleConstants moduleConstants[4], double wheelBase)
{
    m_Gyro = CowPigeon::GetInstance();

    m_Locked = false;

    for (int i = 0; i < 4; i++)
    {
        m_Modules[i] = new SwerveModule(i,
                                        moduleConstants[i].driveMotorId,
                                        moduleConstants[i].rotationMotorId,
                                        moduleConstants[i].encoderId,
                                        moduleConstants[i].encoderOffset);
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
                              double centerOfRotationY)
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

    auto moduleStates = m_Kinematics->CalculateModuleStates(chassisSpeeds, centerOfRotationX, centerOfRotationY);

    // This just overwrites for now. Maybe fix?
    if (m_Locked)
    {
        double angles[4]{ 45, 315, 135, 225 };
        // TODO: check if 0.3 is good
        // units::feet_per_second_t velocity { 0.3 };

        for (int i = 0; i < 4; i++)
        {
            moduleStates[i] = CowLib::CowSwerveModuleState{ 0.3, angles[i] };
        }
    }

    // Just in case
    CowLib::CowSwerveKinematics::DesaturateSpeeds(&moduleStates, CONSTANT("SWERVE_MAX_SPEED"));

    for (auto module : m_Modules)
    {
        module->SetTargetState(moduleStates[module->GetId()]);
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
                              double centerOfRotationY)
{
    SetVelocity(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega, isFieldRelative);
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
 *
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
    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions;
    for (auto module : m_Modules)
    {
        modulePositions[module->GetId()] = module->GetPosition();
    }

    m_Odometry->Reset(pose, pose.Rotation().Degrees().value(), modulePositions);
    m_Gyro->SetYaw(pose.Rotation().Degrees().value());
}

void SwerveDrive::Handle()
{
    for (auto module : m_Modules)
    {
        module->Handle();
    }

    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions{};
    std::transform(m_Modules.begin(),
                   m_Modules.end(),
                   modulePositions.begin(),
                   [](SwerveModule *module) { return module->GetPosition(); });

    m_Odometry->Update(m_Gyro->GetYawDegrees(), modulePositions);

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "odometry pose: x %f, y %f, %fdeg",
    //                           m_Odometry->GetX(),
    //                           m_Odometry->GetY(),
    //                           m_Odometry->GetRotation());

    // frc::SmartDashboard::PutNumberArray("odometry pose (x, y, deg)",
    //                                     { m_Odometry->GetX(), m_Odometry->GetY(), m_Odometry->GetRotation() });
}
