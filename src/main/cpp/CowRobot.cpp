#include "CowRobot.h"

#include "CowLib/CowLogger.h"
#include "Subsystems/Limelight.h"
#include "Subsystems/Vision.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    // uncomment for b-bot
    m_PowerDistributionPanel = new frc::PowerDistribution(1, frc::PowerDistribution::ModuleType::kRev);
    // m_PowerDistributionPanel = new frc::PowerDistribution();

    // mxp board was removed from robot - can remove this code
    m_LEDDisplay = nullptr;

    m_Gyro = CowPigeon::GetInstance();

    m_PreviousGyroError = 0;
    // m_Gyro->Reset(); - don't know why we have this commented
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::Accelerometer::kRange_4G);

    // Set up drivetrain
    // TODO: reset constants needs to reset this
    // fl, fr, bl, br
    // drive motor, angle motor, encoder canId's
    SwerveDrive::ModuleConstants swerveModuleConstants[4]{
        SwerveDrive::ModuleConstants{ 4, 3, 26, CONSTANT("SWERVE_FL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 6, 5, 27, CONSTANT("SWERVE_FR_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 2, 1, 25, CONSTANT("SWERVE_BL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 8, 7, 28, CONSTANT("SWERVE_BR_ENCODER_OFFSET") }
    };

    m_Drivetrain = new SwerveDrive(swerveModuleConstants, CONSTANT("WHEEL_BASE"));

    m_Drivetrain->ResetEncoders();

    m_DriveController = new SwerveDriveController(*m_Drivetrain);

    // m_Arm = new Arm(9, 10, 11, 12, 1);
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_PreviousGyroError = 0;

    m_Drivetrain->ResetConstants();
    m_DriveController->ResetConstants();
    // m_Controller->ResetConstants(); error

    Vision::GetInstance()->Reset();

    CowLib::CowLogger::GetInstance()->Reset();
}

/**
 * @brief
 *
 * @param controller
 */
void CowRobot::SetController(GenericController *controller)
{
    m_Controller = controller;
}

void CowRobot::PrintToDS()
{
    if (m_DSUpdateCount++ % 10 == 0)
    {
        m_DSUpdateCount = 1;
    }
}

// Used to handle the recurring logic funtions inside the robot.
// Please call this once per update cycle.
void CowRobot::Handle()
{
    m_MatchTime = CowLib::CowTimer::GetFPGATimestamp() - m_StartTime;

    if (m_Controller == nullptr)
    {
        printf("No controller for CowRobot!!\n");
        return;
    }

    m_Controller->Handle(this);
    m_Drivetrain->Handle();
    m_Arm->Handle();

    // logger code below should have checks for debug mode before sending out data
    CowLib::CowLogger::GetInstance()->Handle();
    // log the following every 200 ms
    if (m_DSUpdateCount % 10 == 0)
    {
        // m_DSUpdateCount is reset in PrintToDS
        CowLib::CowLogger::LogGyro(m_Gyro->GetPitchDegrees(), m_Gyro->GetRollDegrees(), m_Gyro->GetYawDegrees());
        CowLib::CowLogger::LogPose(m_Drivetrain->GetPoseX(), m_Drivetrain->GetPoseY(), m_Drivetrain->GetPoseRot());
    }

    // accelerometers
    double zVal = m_ZFilter.Calculate(m_Accelerometer->GetZ());
    // positive is true, negative is false
    // bool direction = (zVal - m_PrevZ) > 0 ? true : false;
    m_PrevZ = zVal;

    PrintToDS();
}

void CowRobot::StartTime()
{
    m_StartTime = CowLib::CowTimer::GetFPGATimestamp();
}

void CowRobot::DoNothing()
{
    // TODO: make the robot stop (including drive)
}

double CowRobot::PIDToAprilTagTranslation()
{
    // return 0.0;
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable("limelight")
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "April tag targeting array has incorrect length");
    }
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "vec length: %d", targetPoseVec.size());
    double targetX = targetPoseVec[0];

    double yOutput = m_AprilTagPIDController.Calculate(0.0, targetX);

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetX, yOutput);
    return yOutput;
}

double CowRobot::PIDToAprilTagRotation()
{
    // double test = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "test LL %f", test);

    return 0.0;
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }
    auto targetPoseVec = LimelightHelpers::GetTargetPose_RobotSpace();
    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "April tag targeting array has incorrect length");
    }
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "vec length: %d", targetPoseVec.size());

    double targetYaw = targetPoseVec[4];

    double yawOutput = m_AprilTagPIDController.Calculate(0.0, targetYaw);

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag yaw target: %f output: %f", targetYaw, yawOutput);

    return yawOutput;
}

/**
 * @brief Updates arm state based on inputs from operator
 * 
 */
void CowRobot::SetArmState(ARM_STATE state, ARM_CARGO cargo)
{
    m_Arm->SetArmState(state);

    if (state == ARM_IN)
    {
        m_Arm->SetArmCargo(cargo);
    }
}

/**
 * called each cycle by operator controller (at the bottom)
*/
void CowRobot::ArmSM()
{
    switch (m_Arm->GetArmState())
    {
    case ARM_NONE :
        // m_Arm->SetAngle(0);
        // m_Arm->SetTelescopePosition(0);
        m_Arm->UpdateClawState();
        break;
    case ARM_IN :
        m_Arm->UpdateClawState();
        break;
    case ARM_STOW :
        m_Arm->UpdateClawState();
        break;
    case ARM_L3 :
        break;
    case ARM_L2 :
        break;
    case ARM_L1 :
        break;
    case ARM_SCORE :
        break;
    case ARM_MANUAL :
        break;
    default :
        break;
    }
}
