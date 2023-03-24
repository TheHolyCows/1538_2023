#include "CowPigeon.h"

CowPigeon *CowPigeon::s_Instance = nullptr;

CowPigeon *CowPigeon::GetInstance()
{
    if (s_Instance == nullptr)
    {
        s_Instance = new CowPigeon();
    }

    return s_Instance;
}

CowPigeon::CowPigeon()
{
    constexpr int PIGEON_ID = 24;

    m_Pigeon = new ctre::phoenixpro::hardware::Pigeon2(PIGEON_ID, "cowdrive");

    m_Inverted = false;
}

void CowPigeon::SetInverted(bool inverted)
{
    m_Inverted = inverted;
}

units::degree_t CowPigeon::GetYaw()
{
    return m_Pigeon->GetYaw().Refresh().GetValue() * (m_Inverted ? -1 : 1);
}

units::degree_t CowPigeon::GetPitch()
{
    return m_Pigeon->GetPitch().Refresh().GetValue() * (m_Inverted ? -1 : 1);
}

units::degree_t CowPigeon::GetRoll()
{
    return m_Pigeon->GetRoll().Refresh().GetValue() * (m_Inverted ? -1 : 1);
}

double CowPigeon::GetYawDegrees()
{
    return GetYaw().value();
}

double CowPigeon::GetPitchDegrees()
{
    return GetPitch().value();
}

double CowPigeon::GetRollDegrees()
{
    return GetRoll().value();
}

void CowPigeon::SetYaw(units::degree_t angle)
{
    m_Pigeon->SetYaw(angle);
}

void CowPigeon::SetYaw(double angle)
{
    SetYaw(units::degree_t{ angle });
}
