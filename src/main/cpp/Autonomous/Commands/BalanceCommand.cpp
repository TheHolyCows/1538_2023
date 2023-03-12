#include "BalanceCommand.h"

BalanceCommand::BalanceCommand(const double speed, const double timeout, const double maxDistance, const bool pitchOnly)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Speed(speed),
      m_Timeout(timeout),
      m_MaxDistance(maxDistance),
      m_PitchOnly(pitchOnly),
      m_GyroLPF(frc::LinearFilter<double>::MovingAverage(10)),
      m_AccelerometerLPF(frc::LinearFilter<double>::MovingAverage(10))
{
    m_OnIncline     = false;
    m_SecondIncline = false;
    m_LastPitch     = 0;
    m_Done          = false;
    m_Sign          = 1;
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
    double pitch = m_GyroLPF.Calculate(m_Gyro.GetPitchDegrees());
    double accel = m_AccelerometerLPF.Calculate(m_Accelerometer.GetZ());
    // double err   = pitch - m_LastPitch;

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "accel : %f", accel);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "pitch : %f", pitch);

    if (!m_PitchOnly && (fabs(pitch) > CONSTANT("CHARGE_INIT_THRESHOLD") && fabs(accel) > 1))
    {
        m_Done = true;
        return;
    }
    else if (m_PitchOnly)
    {
        if (fabs(pitch) < 1 && fabs(pitch) > 0)
        {
            m_Done = true;
            return;
        }
        if (fabs(pitch) < CONSTANT("CHARGE_TIP_THRESHOLD") && fabs(pitch) < fabs(m_LastPitch) - 0.1)
        {
            m_Done = true;
            return;
        }
    }

    robot->GetDrivetrain()->SetLocked(false);
    robot->GetDrivetrain()->SetVelocity(m_Speed, 0, 0, true);

    m_LastPitch = pitch;
}

void BalanceCommand::Finish(CowRobot *robot)
{
    if (!m_PitchOnly)
    {
        robot->GetDrivetrain()->SetLocked(false);
    }
    else
    {
        robot->GetDrivetrain()->SetLocked(true);
    }
    robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
}