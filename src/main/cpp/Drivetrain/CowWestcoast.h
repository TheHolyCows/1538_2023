// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // CowWestcoast.h
// // author: jon-bassi
// //==================================================

// #ifndef __COW_WESTCOAST_H__
// #define __COW_WESTCOAST_H__

// #include "../CowConstants.h"
// #include "../CowLib/CowMotorController.h"
// #include "../CowLib/Utility.h"
// #include "../CowPigeon.h"

// namespace Drivetrain
// {
//     /**
//      * classic tank style drive
//      * 4 motors, 2 each side, running 6 wheels
//      */
//     class CowWestcoast
//     {
//     public:
//         CowWestcoast(int driveLeftA, int driveLeftB, int driveRightA, int driveRightB);
//         ~CowWestcoast();

//         double getDriveDistance(void);

//         void setNeutralMode(CowLib::CowMotorController::CowNeutralMode);
//         void setStatorLimits(double limit, double threshold, double duration);
//         void setMotors(double left, double right);

//         void teleopDrive(double throttle, double angle, bool quickturn);
//         bool pidDistanceHeadingDrive(double distance, double heading, double speed);
//         bool pidHeadingDrive(double heading, double speed);

//         void reset(void);
//         void handle(void);

//     private:
//         CowLib::CowMotorController *m_LeftDriveA;
//         CowLib::CowMotorController *m_LeftDriveB;
//         CowLib::CowMotorController *m_RightDriveA;
//         CowLib::CowMotorController *m_RightDriveB;

//         // current percent throttle
//         double m_LeftDriveValue  = 0;
//         double m_RightDriveValue = 0;

//         // pid distance error
//         double m_PreviousDriveError = 0;

//         // gyro variables for pid heading calculation
//         CowPigeon *m_Gyro;
//         double m_PreviousGyroError = 0;
//     };
// } // namespace Drivetrain

// #endif /* __COW_WESTCOAST_H__ */