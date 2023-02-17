#include "CowRobot.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    m_PowerDistributionPanel = new frc::PowerDistribution();

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

double CowRobot::YPIDOutputToAprilTag()
{
    return 0.0;
}

/**
 * called each cycle by operator controller (at the bottom)
*/
void CowRobot::ArmSM()
{
    switch (m_Arm->GetArmState())
    {
    case ARM_NONE :
        m_Arm->SetAngle(0);
        m_Arm->SetTelescopePosition(0);
        break;
    case ARM_IN :
        break;
    case ARM_STOW :
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
