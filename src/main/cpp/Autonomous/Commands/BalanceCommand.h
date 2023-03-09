#pragma once

#include "../../CowConstants.h"
#include "../../CowLib/CowLogger.h"
#include "../../CowLib/CowTimer.h"
#include "../../CowPigeon.h"
#include "RobotCommand.h"

#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose2d.h>

class BalanceCommand : public RobotCommand
{
public:
    BalanceCommand(double speed, double timeout, double maxDistance, bool pitchOnly);
    ~BalanceCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;
    CowPigeon &m_Gyro;

    const double m_Speed;
    const double m_Timeout;
    const double m_MaxDistance;
    const bool m_PitchOnly;

    frc::Pose2d m_StartingPose;

    frc::LinearFilter<double> m_GyroLPF;
    frc::LinearFilter<double> m_AccelerometerLPF;

    frc::BuiltInAccelerometer m_Accelerometer{};

    bool m_OnIncline;
    bool m_SecondIncline;

    double m_LastPitch;

    int m_Sign;

    bool m_Done;
};