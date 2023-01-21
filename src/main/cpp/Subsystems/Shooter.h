// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // Shooter.h
// // author: ssemtner
// // created on: 2022-2-12
// //==================================================

// #ifndef __SRC_SUBSYSTEMS_SHOOTER_H__
// #define __SRC_SUBSYSTEMS_SHOOTER_H__

// #include "../CowConstants.h"
// #include "../CowLib/CowLogger.h"
// #include "../CowLib/CowLPF.h"
// #include "../CowLib/CowMotorController.h"

// #include <algorithm>
// #include <string>

// class Shooter
// {
// public:
//     Shooter(int motorControllerS1, int motorControllerS2, int motorControllerH, int motorControllerR);
//     bool AtTarget();
//     void SetSpeed(double speedF);
//     void SetSpeedHoodRelative(void);
//     void SetHoodRollerSpeed(double speed);
//     void SetHoodPosition(double position);
//     void SetHoodPositionUp(void);
//     void SetHoodPositionDown(void);
//     void SetHoodPositionBottom(void);
//     void ZeroHoodPosition(void);

//     double GetSetpointF() { return m_Setpoint; }

//     double GetSetpointR() { return m_SetpointRoller; }

//     double GetSetpointH() { return m_HoodPosition; }

//     void ResetConstants();

//     void SetClosedLoopError(int error)
//     {
//         if (m_MotorShooter1)
//         {
//             m_MotorShooter1->SetClosedLoopError(error);
//         }
//     }

//     void handle();

//     double CalcShooterTolerance(void);

//     double GetSpeedF();
//     double GetSpeedRoller();
//     double GetHoodPosition();

//     virtual ~Shooter();

// private:
//     CowLib::CowMotorController *m_MotorShooter1;
//     CowLib::CowMotorController *m_MotorShooter2;
//     CowLib::CowMotorController *m_MotorHood;
//     CowLib::CowMotorController *m_MotorHoodRoller;
//     CowLib::CowLPF *m_RampLPF_F;

//     int m_Motor1ID;

//     double m_SpeedShooter;
//     double m_Setpoint;

//     double m_HoodRollerSpeed;
//     double m_SetpointRoller;

//     double m_HoodPosition;
//     double m_HoodUpLimit;
//     double m_HoodDownLimit;

//     bool m_ZeroingHood;
//     bool m_HoodZeroed;

//     CowLib::CowLogger *m_LogServer;
// };

// #endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
