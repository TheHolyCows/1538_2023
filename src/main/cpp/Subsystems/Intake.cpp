// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // Intake.cpp
// // author: ssemtner
// // created on: 2022-2-12
// //==================================================

// #include "Intake.h"

// #include <frc/Solenoid.h>
// #include <frc/Timer.h>
// #include <iostream>

// Intake::Intake(int intakeMotor, int indexMotor, int solenoidChannelA, double scale)
// {
//     m_MotorIntake = new CowLib::CowMotorController(intakeMotor);
//     m_MotorIndex  = new CowLib::CowMotorController(indexMotor);

//     m_Scale = scale;

//     m_Solenoid    = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, solenoidChannelA);
//     m_IntakeSpeed = 0;
//     m_IndexSpeed  = 0;

//     SetStatusFramePeriod();
// }

// void Intake::SetSpeed(double intakeSpeed, double indexSpeed)
// {
//     m_IntakeSpeed = intakeSpeed * m_Scale;
//     m_IndexSpeed  = indexSpeed * m_Scale;
// }

// void Intake::SetIntakeSpeed(double speed)
// {
//     m_IntakeSpeed = speed * m_Scale;
// }

// void Intake::SetIndexSpeed(double speed)
// {
//     m_IndexSpeed = speed * m_Scale;
// }

// void Intake::SetExtended(bool extended)
// {
//     m_IntakeExtended = extended;
// }

// void Intake::SetStatusFramePeriod()
// {
//     m_MotorIntake->GetInternalMotor()->SetStatusFramePeriod(Status_1_General, 40);
//     m_MotorIntake->GetInternalMotor()->SetStatusFramePeriod(Status_2_Feedback0, 80);

//     m_MotorIndex->GetInternalMotor()->SetStatusFramePeriod(Status_1_General, 40);
//     m_MotorIndex->GetInternalMotor()->SetStatusFramePeriod(Status_2_Feedback0, 80);
// }

// void Intake::handle()
// {
//     // Pneumatics
//     m_Solenoid->Set(m_IntakeExtended);

//     // Intake
//     m_MotorIntake->Set(m_IntakeSpeed);

//     // Indexer - because we may want to run this independently...
//     m_MotorIndex->Set(m_IndexSpeed);
// }

// Intake::~Intake()
// {
//     delete m_MotorIntake;
//     delete m_MotorIndex;
//     delete m_Solenoid;
//     // delete m_SolenoidB;
// }
