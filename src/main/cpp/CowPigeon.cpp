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

    m_Pigeon = new ctre::phoenix::sensors::Pigeon2(PIGEON_ID);

    m_Inverted = false;
}

double CowPigeon::GetYaw()
{
    return GetRawYaw() * (m_Inverted ? -1 : 1);
}

double CowPigeon::GetPitch()
{
    return GetRawPitch() * (m_Inverted ? -1 : 1);
}

double CowPigeon::GetRoll()
{
    return GetRawRoll() * (m_Inverted ? -1 : 1);
}

double CowPigeon::GetRawYaw()
{
    return m_Pigeon->GetYaw();
}

double CowPigeon::GetRawPitch()
{
    return m_Pigeon->GetPitch();
}

double CowPigeon::GetRawRoll()
{
    return m_Pigeon->GetRoll();
}
