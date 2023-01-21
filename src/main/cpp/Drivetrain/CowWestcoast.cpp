// //==================================================
// // Copyright (C) 2022 Team 1538 / The Holy Cows
// // CowWestcoast.cpp
// // author: jon-bassi
// //==================================================

// #include "CowWestcoast.h"

// namespace Drivetrain
// {
//     /**
//      * @brief Construct a new Cow Westcoast:: Cow Westcoast object
//      * TODO: add in calculations for drive distance
//      * @param driveLeftA
//      * @param driveLeftB
//      * @param driveRightA
//      * @param driveRightB
//      */
//     CowWestcoast::CowWestcoast(int driveLeftA, int driveLeftB, int driveRightA, int driveRightB)
//     {
//         // automaitically set to percentvbus and coast in motor controller class
//         m_LeftDriveA  = new CowLib::CowMotorController(driveLeftA);
//         m_LeftDriveB  = new CowLib::CowMotorController(driveLeftB);
//         m_RightDriveA = new CowLib::CowMotorController(driveRightA);
//         m_RightDriveB = new CowLib::CowMotorController(driveRightB);

//         // inversion settings - need to test but theoretically should keep us from throwing a bunch of negative
//         // signs in our calculations
//         // false = counter clockwise
//         // true  = clockwise
//         m_LeftDriveA->SetInverted(false);
//         m_LeftDriveB->SetInverted(false);
//         m_RightDriveA->SetInverted(true);
//         m_RightDriveB->SetInverted(true);
//         // could additionally set master-follower relationship here for left and right drives

//         // 2022 constants:
//         // STATOR_LIMIT = 80; STATOR_THRESHOLD = 95; STATOR_DURATION = 0.28;
//         setStatorLimits(CONSTANT("STATOR_LIMIT"), CONSTANT("STATOR_THRESHOLD"), CONSTANT("STATOR_DURATION"));

//         // get reference to gyro for PID heading calculations
//         m_Gyro = CowPigeon::GetInstance();
//     }

//     double CowWestcoast::getDriveDistance(void)
//     {
//         double direction = -1.0;
//         double position  = 0;
//         if (m_LeftDriveA)
//         {
//             // position / falcon units per rev * gear ratio
//             position = m_LeftDriveA->GetPosition() / (2048 * (46 / 12));
//             position *= 12.56636;
//             position *= direction;
//         }
//         return position;
//     }

//     void CowWestcoast::setNeutralMode(CowLib::CowMotorController::CowNeutralMode mode)
//     {
//         m_LeftDriveA->SetNeutralMode(mode);
//         m_LeftDriveB->SetNeutralMode(mode);
//         m_RightDriveA->SetNeutralMode(mode);
//         m_RightDriveB->SetNeutralMode(mode);
//     }

//     /**
//      * @brief stator limiting helps to make the motor more efficient
//      *
//      * @param limit
//      * @param threshold
//      * @param duration
//      */
//     void CowWestcoast::setStatorLimits(double limit, double threshold, double duration)
//     {
//         m_LeftDriveA->SetStatorLimit(limit, threshold, duration);
//         m_LeftDriveB->SetStatorLimit(limit, threshold, duration);
//         m_RightDriveA->SetStatorLimit(limit, threshold, duration);
//         m_RightDriveB->SetStatorLimit(limit, threshold, duration);
//     }

//     /**
//      * @brief standard yaw + throttle west coast drive code
//      *        does not PID
//      *        reimplementation of DriveSpeedTurn()
//      * @param throttle -1.0 -> 1.0 reverse/forward (throttle/gamepad y-axis)
//      * @param yaw -1.0 -> 1.0 left/right (steering wheel/gamepad x-axis)
//      * @param quickTurn if true increase yawMultiplier
//      */
//     void CowWestcoast::teleopDrive(double throttle, double yaw, bool quickTurn)
//     {
//         // Linear degredation of steeering based off of throttle

//         // velocity *= 0.003;
//         // double tmpThrottle = throttle;
//         double yawMultiplier = 0;

//         // y-axis deadzone
//         if (throttle < 0.10 && throttle > -0.10)
//             throttle = 0;

//         // x-axis/steering wheel deadzone and quickturn check
//         // prevents spin in place?
//         if (((yaw < 0.02) && (yaw > -0.02)) || ((throttle == 0) && !quickTurn))
//             yaw = 0;

//         // 2022 constants - STEERING_NOQUICKTURN = 0.18; STEERING_QUICKTURN = 0.9;
//         if (quickTurn)
//         {
//             if (throttle == 0.0)
//             {
//                 yawMultiplier = 1;
//             }
//             else
//             {
//                 yawMultiplier = CONSTANT("STEERING_QUICKTURN");
//             }
//         }
//         else
//         {
//             yawMultiplier = CONSTANT("STEERING_NOQUICKTURN");
//         }

//         yaw *= yawMultiplier;

//         double leftThrottle  = throttle - yaw;
//         double rightThrottle = throttle + yaw;

//         setMotors(leftThrottle, rightThrottle);
//     }

//     /**
//      * @brief PIDs the robot to a specific distance + heading
//      *        typically used with auto modes. I believe this attempts
//      *        to drive exactly to the distance specified rather than to
//      *        at least that distance
//      *        reimplementation of DriveDistanceWithHeading()
//      * @param distance distance to PID to
//      * @param heading heading to PID to
//      * @param throttle percent throttle to run robot at
//      * @return double
//      */
//     bool CowWestcoast::pidDistanceHeadingDrive(double distance, double heading, double throttle)
//     {
//         // distance error calculation
//         double error  = distance - getDriveDistance();
//         double dError = error - m_PreviousDriveError;

//         // 2022 constants - DRIVE_P = 0.052; DRIVE_D = 0.04;
//         double output = CONSTANT("DRIVE_P") * error + CONSTANT("DRIVE_D") * dError;

//         throttle = CowLib::LimitMix(output, throttle);

//         bool headingResult = pidHeadingDrive(heading, throttle);

//         m_PreviousDriveError = error;
//         return (fabs(error) < 4 && headingResult);
//     }

//     /**
//      * @brief PIDs the robot to a specific heading - typically used with auto mode
//      *        reimplementation of DriveWithHeading()
//      * @param heading heading to PID to
//      * @param throttle percent throttle to run robot at
//      * @return true
//      * @return false
//      */
//     bool CowWestcoast::pidHeadingDrive(double heading, double throttle)
//     {
//         // heading error calculation
//         double error  = m_Gyro->GetYawDegrees() - heading;
//         double dError = error - m_PreviousGyroError;

//         // 2022 constants - TURN_P = 0.015; TURN_D = 0.045;
//         double output = CONSTANT("TURN_P") * error + CONSTANT("TURN_D") * dError;

//         setMotors(throttle - output, throttle + output);

//         m_PreviousGyroError = error;
//         return (fabs(error) < 1 && CowLib::UnitsPerSecond(fabs(dError)) < 0.5);
//     }

//     /**
//      * @brief sets throttle % for left and right motors
//      *        reimplementation of DriveLeftRight()
//      * @param left % throttle for left motors
//      * @param right % throttle for right motors
//      */
//     void CowWestcoast::setMotors(double left, double right)
//     {
//         m_LeftDriveValue  = CowLib::LimitMix(left);
//         m_RightDriveValue = CowLib::LimitMix(right);
//     }

//     void CowWestcoast::handle(void)
//     {
//         m_LeftDriveA->Set(m_LeftDriveValue);
//         m_LeftDriveB->Set(m_LeftDriveValue);
//         m_RightDriveA->Set(m_RightDriveValue);
//         m_RightDriveB->Set(m_RightDriveValue);
//     }

//     void CowWestcoast::reset(void)
//     {
//         m_LeftDriveValue  = 0;
//         m_RightDriveValue = 0;

//         m_PreviousDriveError = 0;
//         m_PreviousGyroError  = 0;

//         m_LeftDriveA->SetSensorPosition(0);
//         m_LeftDriveB->SetSensorPosition(0);
//         m_RightDriveA->SetSensorPosition(0);
//         m_RightDriveB->SetSensorPosition(0);
//     }
// } // namespace Drivetrain