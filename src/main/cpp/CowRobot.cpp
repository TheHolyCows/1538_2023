#include "CowRobot.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    m_LEDDisplay = new CowLib::CowAlphaNum(0x70);

    m_PowerDistributionPanel = new frc::PowerDistribution();

    m_Gyro = CowLib::CowGyro::GetInstance();

    m_PreviousGyroError = 0;
    // m_Gyro->Reset(); - don't know why we have this commented
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::Accelerometer::kRange_4G);

    // Set up drivetrain
    // TODO: reset constants needs to reset this
    // fl, fr, bl, br
    SwerveDrive::ModuleConstants swerveModuleConstants[4]{
        SwerveDrive::ModuleConstants{ 1, 2, 9, 2 },
        SwerveDrive::ModuleConstants{ 3, 4, 10, 2 },
        SwerveDrive::ModuleConstants{ 5, 6, 11, -2 },
        SwerveDrive::ModuleConstants{ 7, 8, 12, -2 }
    };

    m_Drivetrain = new SwerveDrive(swerveModuleConstants, CONSTANT("WHEEL_BASE"));

    m_Shooter = new Shooter(11, 13, 12, 14);
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_PreviousGyroError = 0;

    m_Drivetrain->Reset();

    m_Shooter->ResetConstants();
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
    if (m_DSUpdateCount % 10 == 0)
    {
        m_DSUpdateCount = 0;
    }
}

// Used to handle the recurring logic funtions inside the robot.
// Please call this once per update cycle.
void CowRobot::handle()
{
    m_MatchTime = CowLib::CowTimer::GetFPGATimestamp() - m_StartTime;

    if (m_Controller == NULL)
    {
        printf("No controller for CowRobot!!\n");
        return;
    }

    m_Controller->Handle(this);
    m_Drivetrain->Handle();
    m_Shooter->handle();

    // accelerometers
    double zVal = m_ZFilter.Calculate(m_Accelerometer->GetZ());
    // positive is true, negative is false
    bool direction = (zVal - m_PrevZ) > 0 ? true : false;
    m_PrevZ        = zVal;

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
