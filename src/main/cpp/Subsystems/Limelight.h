// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // Limelight.h
// // author: jon-bassi
// // created on: 2022-2-28
// //==================================================

// #ifndef __LIMELIGHT_H__
// #define __LIMELIGHT_H__

#include "../CowConstants.h"
#include "../CowLib/CowLogger.h"

#include <algorithm>
#include <cmath>
#include <frc/filter/MedianFilter.h>
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>

// class Limelight
// {
// public:
//     Limelight(std::string hostname);

//     enum LimelightMode
//     {
//         LIMELIGHT_TRACKING = 0,
//         LIMELIGHT_VISUAL
//     };

<<<<<<< HEAD
void SetPipeline(int);
frc::Pose2d GetPose(void);

// void SetMode(LimelightMode);
// bool GetValidTargets(void);
// float GetTargetXPos(void);
// void ClearXFilter(void);
// float GetTargetYPos(void);
// void ClearYFilter(void);

// bool TargetCentered(void);

// int CalcHoodPos(void);
// float CalcYPercent(void);
// float CalcNewPid(void);

// void ResetPID(void);
=======
//     void SetMode(LimelightMode);
//     bool GetValidTargets(void);
//     float GetTargetXPos(void);
//     void ClearXFilter(void);
//     float GetTargetYPos(void);
//     void ClearYFilter(void);

//     bool TargetCentered(void);

//     int CalcHoodPos(void);
//     float CalcYPercent(void);
//     float CalcNewPid(void);

//     void ResetPID(void);
>>>>>>> develop

// private:
//     std::shared_ptr<nt::NetworkTable> m_Limelight;
//     float m_Limelight_PID_P;
//     float m_Limelight_PID_D;

//     frc::MedianFilter<float> m_XFilter;
//     frc::MedianFilter<float> m_YFilter;
// };

// #endif /* __LIMELIGHT_H__ */
