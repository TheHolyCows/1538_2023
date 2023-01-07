//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __DECLARATIONS_H__
#define __DECLARATIONS_H__

#define ROBOT_HZ 400

#define COWCONSTANTS_DEFAULT_FILE "/home/lvuser/constants.ini"

// Number of accumulation periods to be summed
#define COWGYRO_RING_SIZE 5
// Length of an accumulation period in seconds
#define COWGYRO_ACCUMULATION_PERIOD 1.0

#define GYRO_SENSITIVITY 0.007

// CAN IDs
#define DRIVE_RIGHT_A 3
#define DRIVE_RIGHT_B 4
#define DRIVE_LEFT_A 1
#define DRIVE_LEFT_B 2

#define MXP_QEI_1_A 10
#define MXP_QEI_1_B 11
#define MXP_QEI_2_A 12
#define MXP_QEI_2_B 13
#define MXP_QEI_3_A 18
#define MXP_QEI_3_B 19
#define MXP_QEI_4_A 20
#define MXP_QEI_4_B 21
#define MXP_QEI_5_A 22
#define MXP_QEI_5_B 23

// Analog inputs
#define ANALOG_GYRO 0

// Digital outputs
#define RELAY_COMPRESSOR 1

// Digital inputs
#define DIGITAL_PRESSURE_SWITCH 1

#define SOLENOID_PTO 7
#define SOLENOID_ARM 6
#define SOLENOID_LOCK 5
#define SOLENOID_UNLOCK 4

#endif
