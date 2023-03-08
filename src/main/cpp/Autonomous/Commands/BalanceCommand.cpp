#include "BalanceCommand.h"

BalanceCommand::BalanceCommand(const double speed, const double timeout, const double maxDistance)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Speed(speed),
      m_Timeout(timeout),
      m_MaxDistance(maxDistance),
      m_OnIncline(false),
      m_LastPitch(0),
      m_Done(false)
{
}

bool BalanceCommand::IsComplete(CowRobot *robot)
{
    if (m_Timer->HasElapsed(m_Timeout) || m_Done)
    {
        return true;
    }

    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();
    double distance = currentPose.Translation().Distance(m_StartingPose.Translation()).convert<units::foot>().value();

    if (distance > m_MaxDistance)
    {
        return true;
    }

    return false;
}

void BalanceCommand::Start(CowRobot *robot)
{
    m_Timer->Start();

    m_StartingPose = robot->GetDrivetrain()->GetPose();
}

void BalanceCommand::Handle(CowRobot *robot)
{
    double pitch = m_Gyro.GetPitchDegrees();
    double err = fabs(pitch - m_LastPitch);

    if (m_OnIncline)
    {
        if (err > CONSTANT("BALANCE_PITCH_THRESHOLD"))
        {
            m_Done = true;
            return;
        }

        robot->GetDrivetrain()->SetLocked(false); // Make sure we aren't locked
        robot->GetDrivetrain()->SetVelocity(m_Speed, 0, 0, true);
    }
    else
    {
        if (err > CONSTANT("BALANCE_PITCH_THRESHOLD"))
        {
            m_OnIncline = true;
            return;
        }
        else
        {
            robot->GetDrivetrain()->SetLocked(false);
            robot->GetDrivetrain()->SetVelocity(m_Speed, 0, 0, true);
        }
    }

    m_LastPitch = pitch;
}

void BalanceCommand::Finish(CowRobot *robot)
{
    robot->GetDrivetrain()->SetLocked(true);
    robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
}