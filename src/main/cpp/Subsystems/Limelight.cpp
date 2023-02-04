//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// Limelight.cpp
// author: jon-bassi
// created on: 2022-2-28
//==================================================

#include "Limelight.h"

Limelight::Limelight(std::string hostname)
    : m_XFilter(CONSTANT("X_FILTER_SIZE")),
      m_YFilter(CONSTANT("Y_FILTER_SIZE"))
{
    m_Limelight = nt::NetworkTableInstance::GetDefault().GetTable(hostname);

    // ResetPID();

    // SetPipeline(0);
}

frc::Pose2d Limelight::GetPose()
{
    std::vector<double> poseVector;
    constexpr double defaultVal[]{ 0, 0, 0, 0, 0, 0 };
    poseVector = m_Limelight->GetNumberArray("botpose2d", std::span{ defaultVal });
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                              "limelight translation: %f, %f, %f\nlimelight rotation: %f, %f, %f",
                              poseVector[0],
                              poseVector[1],
                              poseVector[2],
                              poseVector[3],
                              poseVector[4],
                              poseVector[5]);
}

// void Limelight::SetMode(LimelightMode mode)
// {
//     switch (mode)
//     {
//     case LIMELIGHT_TRACKING :
//         m_Limelight->PutNumber("pipeline", 0);
//         m_Limelight->PutNumber("ledMode", 3);
//         break;
//     case LIMELIGHT_VISUAL :
//     default :
//         m_Limelight->PutNumber("pipeline", 3);
//         m_Limelight->PutNumber("ledMode", 1);
//         break;
//     }
// }

// bool Limelight::GetValidTargets()
// {
//     return m_Limelight->GetNumber("tv", 0);
// }

// float Limelight::GetTargetXPos()
// {
//     return m_XFilter.Calculate(m_Limelight->GetNumber("tx", 0.0));
// }

// void Limelight::ClearXFilter()
// {
//     m_XFilter.Reset();
// }

// float Limelight::GetTargetYPos()
// {
//     return m_Limelight->GetNumber("ty", 0.0);
// }

// void Limelight::ClearYFilter()
// {
//     m_YFilter.Reset();
// }

// bool Limelight::TargetCentered()
// {
//     return fabs(m_Limelight->GetNumber("tx", 27.0)) <= 2.0 && GetValidTargets();
// }

// // int Limelight::CalcHoodPos()
// // {
// //     // assuming bigger area = closer
// //     // in the future this target area should be compared to a range of area min and area max in constants
// //     float targArea = m_Limelight->GetNumber("ta", 0.0);

// //     targArea = targArea * CONSTANT("HOOD_DELTA");

// //     // add this number to hood min to get position
// //     return floorf(targArea);
// // }

// float Limelight::CalcYPercent()
// {
//     if (!GetValidTargets())
//     {
//         return 1.0;
//     }

//     // yPercent: 0 is far 1 is close
//     float yTargetingVal = m_YFilter.Calculate(m_Limelight->GetNumber("ty", 28.0));

//     // yPercent probably won't be close to 0 so
//     // Y_TARGETING_FLOOR is the lowest y value we expect the limelight to return
//     // yTargetingRange is the number of values possible
//     float yTargetingRange = 28.0 - CONSTANT("Y_TARGETING_FLOOR");
//     yTargetingVal         = yTargetingVal < CONSTANT("Y_TARGETING_FLOOR") ? CONSTANT("Y_TARGETING_FLOOR") : yTargetingVal;

//     // y% = (value - floor) / total
//     float yPercent = (yTargetingVal - CONSTANT("Y_TARGETING_FLOOR")) / yTargetingRange;

//     return yPercent;
// }

// float Limelight::CalcNewPid()
// {
//     float tmp_limelight_P = -m_Limelight->GetNumber("tx", 0.0);
//     m_Limelight_PID_D     = tmp_limelight_P - m_Limelight_PID_P;
//     m_Limelight_PID_P     = tmp_limelight_P;

//     float pid = (m_Limelight_PID_P * CONSTANT("LIMELIGHT_X_KP"));
//     pid += (m_Limelight_PID_D * CONSTANT("LIMELIGHT_X_KD"));

//     return pid;
// }

// void Limelight::ResetPID()
// {
//     m_Limelight_PID_P = 0;
//     m_Limelight_PID_D = 0;
// }