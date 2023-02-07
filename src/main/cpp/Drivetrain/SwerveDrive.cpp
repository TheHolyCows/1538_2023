#include "SwerveDrive.h"

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/angle.h"
#include "units/angular_velocity.h"

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

    m_SetpointGenerator = new CowLib::CowSwerveSetpointGenerator(m_Kinematics);

    m_Odometry = new CowLib::CowSwerveOdometry(m_Kinematics, CowLib::Pose2d());

    // m_VisionPIDController = new CowLib::CowPID(CONSTANT("SWERVE_VISION_P"), CONSTANT("SWERVE_VISION_I"),
    // CONSTANT("SWERVE_VISION_D"), 0);

    // m_VisionPIDController->SetContinuous(true);
    // m_VisionPIDController->SetInputRange(-180, 180);

    Reset();

    frc::SmartDashboard::PutData("Field", &m_Field);
}

SwerveDrive::~SwerveDrive()
{
    delete m_Odometry;
    delete m_Kinematics;
    delete m_SetpointGenerator;

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

    auto robot_pose_vel = frc::Pose2d(units::foot_t{ chassisSpeeds.vx * 0.02 },
                                      units::foot_t{ chassisSpeeds.vy * 0.02 },
                                      frc::Rotation2d(units::degree_t{ chassisSpeeds.omega * 0.02 }));

    // frc::Twist2d twist_vel = frc::Pose2d{ 0_ft, 0_ft, 0_deg }.Log(robot_pose_vel);

    double dtheta        = robot_pose_vel.Rotation().Radians().value();
    double half_dtheta   = 0.5 * dtheta;
    double cos_minus_one = robot_pose_vel.Rotation().Cos() - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (fabs(cos_minus_one) < 0.000000001)
    {
        halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    }
    else
    {
        halftheta_by_tan_of_halfdtheta = -(half_dtheta * robot_pose_vel.Rotation().Sin()) / cos_minus_one;
    }
    frc::Translation2d translation_part
        = robot_pose_vel.Translation().RotateBy(frc::Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    auto twist_vel = frc::Twist2d{ translation_part.X(), translation_part.Y(), units::radian_t{ dtheta } };

    auto updated_chassis_speeds = CowLib::CowChassisSpeeds{ twist_vel.dx.convert<units::foot>().value() / 0.02,
                                                            twist_vel.dy.convert<units::foot>().value() / 0.02,
                                                            twist_vel.dtheta.convert<units::degree>().value() / 0.02 };

    auto setpoint = m_SetpointGenerator->GenerateSetpoint(
        { CONSTANT("SWERVE_MAX_SPEED"), CONSTANT("MAX_ACCEL"), CONSTANT("SWERVE_MAX_ANGULAR_VELOCITY") },
        m_PrevSetpoint,
        updated_chassis_speeds,
        0.02);

    m_PrevSetpoint = setpoint;

    frc::SmartDashboard::PutNumber("vx new", updated_chassis_speeds.vx);
    frc::SmartDashboard::PutNumber("vy new", updated_chassis_speeds.vy);
    frc::SmartDashboard::PutNumber("omega new", updated_chassis_speeds.omega);
    frc::SmartDashboard::PutNumber("vx old", chassisSpeeds.vx);
    frc::SmartDashboard::PutNumber("vy old", chassisSpeeds.vy);
    frc::SmartDashboard::PutNumber("omega old", chassisSpeeds.omega);

    auto moduleStates = m_Kinematics->CalculateModuleStates(chassisSpeeds, 0, 0);

    //auto moduleStates = setpoint.moduleStates;

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
    // CowLib::CowSwerveKinematics::DesaturateSpeeds(&moduleStates, CONSTANT("SWERVE_MAX_SPEED"));

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
    SetVelocity(chassisSpeeds.vx,
                chassisSpeeds.vy,
                chassisSpeeds.omega,
                isFieldRelative,
                centerOfRotationX,
                centerOfRotationY);
}

/**
 * @brief Get the Pose X value in feet
 * 
 * @return double 
 */
double SwerveDrive::GetPoseX()
{
    return m_Pose.GetX();
}

/**
 * @brief Get the Pose Y value in feet
 * 
 * @return double 
 */
double SwerveDrive::GetPoseY()
{
    return m_Pose.GetY();
}

/**
 * @brief Get the pose rotation value in degrees
 * 
 * @return double 
 */
double SwerveDrive::GetPoseRot()
{
    return m_Pose.GetRotation().GetDegrees();
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

void SwerveDrive::ResetOdometry(CowLib::Pose2d pose)
{
    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions;
    for (auto module : m_Modules)
    {
        modulePositions[module->GetId()] = module->GetPosition();
    }

    m_Odometry->Reset(pose);
    m_Gyro->SetYaw(pose.GetRotation().GetDegrees());
}

void SwerveDrive::Handle()
{
    for (auto module : m_Modules)
    {
        module->Handle();
    }

    std::array<CowLib::CowSwerveModuleState, 4> moduleStates{};
    std::transform(m_Modules.begin(),
                   m_Modules.end(),
                   moduleStates.begin(),
                   [](SwerveModule *module) { return module->GetState(); });

    // printf("Mod 0 state pos: %f, vel: %f", moduleStates[0].position, moduleStates[0].velocity);

    // m_Odometry->Update(m_Gyro->GetYawDegrees(), moduleStates);

    // SIM
    // std::array<frc::SwerveModuleState, 4> moduleStates{};
    // std::transform(m_Modules.begin(),
    //                m_Modules.end(),
    //                moduleStates.begin(),
    //                [](SwerveModule *module) { return module->GetState().ToWPI(); });
    // double gyroAngle = m_Kinematics
    //                        ->CalculateChassisSpeedsWithWheelConstraints(
    //                            { moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3] })
    //                        .omega
    //                    + m_Pose.GetRotation().GetDegrees();
    double gyroAngle = m_Kinematics->CalculateChassisSpeeds(moduleStates).omega + m_Pose.GetRotation().GetDegrees();
    m_Gyro->GetInternalPigeon()->SetYaw(units::degree_t{ gyroAngle });

    m_Pose = m_Odometry->Update(gyroAngle, moduleStates);
    printf("POSE %f %f %f\n", m_Pose.GetX(), m_Pose.GetY(), m_Pose.GetRotation().GetDegrees());

    // m_Pose = m_Odometry->GetWPIPose();
    m_Field.SetRobotPose(frc::Pose2d{ units::foot_t{ m_Pose.GetX() },
                                      units::foot_t{ m_Pose.GetY() },
                                      units::degree_t{ m_Pose.GetRotation().GetDegrees() } });

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "odometry pose: x %f, y %f, %fdeg",
    //                           m_Odometry->GetX(),
    //                           m_Odometry->GetY(),
    //                           m_Odometry->GetRotation());

    // frc::SmartDashboard::PutNumberArray("odometry pose (x, y, deg)",
    //                                     { m_Pose.GetX(), m_Pose.GetY(), m_Pose.GetRotation().GetDegrees() });
}
