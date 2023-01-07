//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowTimer.cpp
// author: jon-bassi
// created on: 2022-1-15
// derived from wpi/Timer.cpp
//==================================================

#include "CowTimer.h"

namespace CowLib
{

    void CowWait(double seconds)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }

    double GetTime()
    {
        using std::chrono::duration;
        using std::chrono::duration_cast;
        using std::chrono::system_clock;

        return duration<double>(system_clock::now().time_since_epoch()).count();
    }

} // namespace CowLib

using namespace CowLib;

CowTimer::CowTimer()
{
    Reset();
}

double CowTimer::Get() const
{
    if (m_Running)
    {
        return (GetFPGATimestamp() - m_StartTime) + m_AccumulatedTime;
    }
    else
    {
        return m_AccumulatedTime;
    }
}

void CowTimer::Reset()
{
    m_AccumulatedTime = 0;
    m_StartTime       = GetFPGATimestamp();
}

void CowTimer::Start()
{
    if (!m_Running)
    {
        m_StartTime = GetFPGATimestamp();
        m_Running   = true;
    }
}

void CowTimer::Stop()
{
    if (m_Running)
    {
        m_AccumulatedTime = Get();
        m_Running         = false;
    }
}

bool CowTimer::HasElapsed(double period) const
{
    return Get() >= period;
}

bool CowTimer::HasPeriodPassed(double period)
{
    return AdvanceIfElapsed(period);
}

bool CowTimer::AdvanceIfElapsed(double period)
{
    if (Get() >= period)
    {
        // Advance the start time by the period.
        m_StartTime += period;
        // Don't set it to the current time... we want to avoid drift.
        return true;
    }
    else
    {
        return false;
    }
}

double CowTimer::GetFPGATimestamp()
{
    // FPGA returns the timestamp in microseconds
    return frc::RobotController::GetFPGATime() * 1.0e-6;
}

double CowTimer::GetMatchTime()
{
    return frc::DriverStation::GetMatchTime();
}
