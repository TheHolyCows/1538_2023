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
    m_Drivetrain = new Drivetrain::CowWestcoast(DRIVE_LEFT_A, DRIVE_LEFT_B, DRIVE_RIGHT_A, DRIVE_RIGHT_B);

    m_Shooter = new Shooter(11, 13, 12, 14);
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_PreviousGyroError = 0;

    m_Drivetrain->reset();

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

    m_Controller->handle(this);
    m_Drivetrain->handle();
    m_Shooter->handle();

    // accelerometers
    double zVal = m_ZFilter.Calculate(m_Accelerometer->GetZ());
    // positive is true, negative is false
    bool direction = (zVal - m_PrevZ) > 0 ? true : false;
    m_PrevZ        = zVal;

    if (m_DSUpdateCount % 10 == 0)
    {
        // driver station logging
    }

    // handle methods for other subsystems go here

    m_DSUpdateCount++;
}

double CowRobot::GetDriveDistance()
{
    return m_Drivetrain->getDriveDistance();
}

// used by autonomous
bool CowRobot::TurnToHeading(double heading)
{
    double PID_P  = CONSTANT("TURN_P");
    double PID_D  = CONSTANT("TURN_D");
    double error  = m_Gyro->GetAngle() - heading;
    double dError = error - m_PreviousGyroError;
    double output = PID_P * error + PID_D * dError;

    // speed *= -speed;

    DriveLeftRight(-output, output);

    m_PreviousGyroError = error;

    return (fabs(error) < 1 && CowLib::UnitsPerSecond(fabs(dError)) < 0.5);
}

// Allows skid steer robot to be driven using tank drive style inputs
// used by most things
void CowRobot::DriveLeftRight(double leftDriveValue, double rightDriveValue)
{
    m_Drivetrain->setMotors(leftDriveValue, rightDriveValue);
}

// Allows robot to spin in place
// unused
void CowRobot::QuickTurn(double turnRate)
{
    // When provided with + turn, quick turn right

    double left  = -1 * turnRate;
    double right = turnRate;

    DriveLeftRight(left, right);
}

void CowRobot::StartTime()
{
    m_StartTime = CowLib::CowTimer::GetFPGATimestamp();
}
