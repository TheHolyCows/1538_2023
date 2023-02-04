// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // Climber.h
// // author: ssemtner
// // created on: 2022-3-17
// //==================================================

// #include "Climber.h"

// Climber::Climber(int leftMotor, int rightMotor)
// {
//     m_LeftMotor  = new CowLib::CowMotorController(leftMotor);
//     m_RightMotor = new CowLib::CowMotorController(rightMotor);

//     m_LeftMotor->SetControlMode(CowLib::CowMotorController::POSITION);
//     m_RightMotor->SetControlMode(CowLib::CowMotorController::POSITION);

//     m_LeftMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
//     m_RightMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

//     m_RightMotor->GetInternalMotor()->SetSensorPhase(true);

//     // need to ensure this works or else bot is f'd
//     while (!m_RightMotor->GetInternalMotor()->GetInverted())
//     {
//         m_RightMotor->GetInternalMotor()->SetInverted(true);
//         CowLib::CowWait(0.1);
//     }
//     m_LeftMotor->GetInternalMotor()->SetInverted(false);
//     // m_LeftMotor->GetInternalMotor()->SetInverted(false);

//     m_LeftPosition  = 0;
//     m_RightPosition = 0;

//     ResetConstants();
// }

// void Climber::SetLeftPosition(double position)
// {
//     m_LeftPosition = position;
// }

// void Climber::SetRightPosition(double position)
// {
//     m_RightPosition = position;
// }

// double Climber::GetLeftPosition()
// {
//     return m_LeftMotor->GetPosition();
// }

// double Climber::GetRightPosition()
// {
//     return m_RightMotor->GetPosition();
// }

// void Climber::NextState()
// {
//     m_State++;
//     ClimberSM();
// }

// void Climber::PrevState()
// {
//     m_State--;
//     if (m_State == 0)
//     {
//         m_State = 5;
//     }
//     ClimberSM();
// }

// /** Untested - DO NOT USE **/
// void Climber::ClimberSM()
// {
//     switch (m_State)
//     {
//     case (NONE) :
//         break;
//     case (EXT_BOTH) :
//         SetRightPosition(CONSTANT("CLIMBER_MID_RUNG"));
//         SetLeftPosition(CONSTANT("CLIMBER_MID_RUNG"));
//         break;
//     case (CLIMB_MID) :
//         SetRightPosition(CONSTANT("CLIMBER_IN"));
//         if (GetRightPosition() < CONSTANT("CLIMBER_OUT") * CONSTANT("CLIMB_DELAY_1"))
//         {
//             m_State++;
//         }
//         break;
//     case (EXT_LEFT_MID) :
//         SetLeftPosition(CONSTANT("CLIMBER_OUT"));
//         // if (direction of swing changes)
//         // {
//         //     m_State++
//         // }
//         break;
//     case (CLIMB_HIGH) :
//         SetLeftPosition(CONSTANT("CLIMBER_IN"));
//         if (GetLeftPosition() < CONSTANT("CLIMBER_OUT") * CONSTANT("CLIMB_DELAY_3"))
//         {
//             m_State++;
//         }
//         break;
//     case (EXT_RIGHT_HIGH) :
//         SetRightPosition(CONSTANT("CLIMBER_OFF_BAR"));
//         if (GetLeftPosition() < CONSTANT("CLIMBER_OUT") * CONSTANT("CLIMB_DELAY_2"))
//         {
//             m_State++;
//         }
//         break;
//     case (EXT_RIGHT_TRAV) :
//         SetRightPosition(CONSTANT("CLIMBER_OUT"));
//         // if (direction of swing changes)
//         // {
//         //     m_State++
//         // }
//         break;
//     case (CLIMB_TRAV) :
//         SetRightPosition(CONSTANT("CLIMBER_MID"));
//         if (GetRightPosition() < CONSTANT("CLIMBER_OUT") * CONSTANT("CLIMB_DELAY_3"))
//         {
//             SetLeftPosition(CONSTANT("CLIMBER_OFF_BAR"));
//         }
//         break;
//     }
// }

// // bool Climber::LeftAtTarget()
// // {
// //     return (fabs(GetLeftPosition() - m_LeftPosition) < CONSTANT("CLIMBER_TOLERANCE"));
// // }

// // bool Climber::RightAtTarget()
// // {
// //     return (fabs(GetRightPosition() - m_RightPosition) < CONSTANT("CLIMBER_TOLERANCE"));
// // }

// void Climber::ResetConstants()
// {
//     m_LeftMotor->SetPIDGains(CONSTANT("CLIMBER_P"), CONSTANT("CLIMBER_I"), CONSTANT("CLIMBER_D"), 0, 1);
//     m_RightMotor->SetPIDGains(CONSTANT("CLIMBER_P"), CONSTANT("CLIMBER_I"), CONSTANT("CLIMBER_D"), 0, 1);
// }

// void Climber::handle()
// {
//     if (m_LeftMotor)
//     {
//         m_LeftMotor->Set(m_LeftPosition);
//     }
//     if (m_RightMotor)
//     {
//         m_RightMotor->Set(m_RightPosition);
//     }
// }

// Climber::~Climber()
// {
//     delete m_LeftMotor;
//     delete m_RightMotor;
// }