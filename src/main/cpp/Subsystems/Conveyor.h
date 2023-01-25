// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // Conveyor.h
// // author: ssemtner
// // created on: 2022-2-12
// //==================================================

// #ifndef SRC_SUBSYSTEMS_CONVEYOR_H_
// #define SRC_SUBSYSTEMS_CONVEYOR_H_

// #include "../CowConstants.h"
// #include "../CowLib/CowLPF.h"
// #include "../CowLib/CowMotorController.h"
// #include "../CowLib/CowPID.h"
// #include "../Declarations.h"

// #include <frc/DigitalInput.h>
// #include <frc/DigitalSource.h>
// #include <frc/DutyCycle.h>
// #include <string>

// class Conveyor
// {
// private:
//     CowLib::CowMotorController *m_MotorUpper;
//     CowLib::CowMotorController *m_MotorFront;
//     CowLib::CowMotorController *m_MotorRear;
//     double m_SpeedUpper;
//     double m_SpeedFront;
//     double m_SpeedRear;
//     int m_MotorRearID;

//     frc::DigitalInput m_ColorSensor;
//     frc::DutyCycle *m_DutyCycle;

// public:
//     enum ConveyorMode
//     {
//         CONVEYOR_OFF     = 0,
//         CONVEYOR_EXHAUST = 1,
//         CONVEYOR_INTAKE  = 2,
//         CONVEYOR_SHOOT   = 3
//     };

//     Conveyor(int upperMotor, int frontMotor, int rearMotor, int colorSensorPinNum);
//     void SetSpeed(double speedUpper, double speedFront, double speedRear);
//     frc::DutyCycle *GetColorSensor(void);

//     void SetStatusFramePeriod(void);

//     void handle();
//     virtual ~Conveyor();
// };

// #endif /* SRC_SUSBSYTEMS_CONVEYOR_H_ */
