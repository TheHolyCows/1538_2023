#include "BalanceCommand.h"

BalanceCommand::BalanceCommand(const double speed, const double timeout, const double maxDistance)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Speed(speed),
      m_Timeout(timeout),
      m_MaxDistance(maxDistance),
      m_PitchHasChanged(false),
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
    if (m_PitchHasChanged)
    {
        // When it started falling from the target
        if (fabs(fabs(m_Gyro.GetPitchDegrees() - fabs(CONSTANT("BALANCE_PITCH_TARGET"))))
            < (CONSTANT("BALANCE_PITCH_TOLERANCE")))
        {
            m_Done = true;
            return;
        }

        robot->GetDrivetrain()->SetLocked(false); // Make sure we aren't locked
        robot->GetDrivetrain()->SetVelocity(m_Speed, 0, 0, true);
    }
    else
    {
        // When it first reaches the target point
        if (fabs(fabs(m_Gyro.GetPitchDegrees() - fabs(CONSTANT("BALANCE_PITCH_TARGET"))))
            < (CONSTANT("BALANCE_PITCH_TOLERANCE")))
        {
            m_PitchHasChanged = true;
        }
        else
        {
            robot->GetDrivetrain()->SetLocked(false);
            robot->GetDrivetrain()->SetVelocity(m_Speed, 0, 0, true);
        }
    }
}

void BalanceCommand::Finish(CowRobot *robot)
{
    robot->GetDrivetrain()->SetLocked(true);
    robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
}