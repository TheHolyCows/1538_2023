//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowMotorController.h"
#include "ArmInterface.h"
#include "ArmState.h"
#include "Claw/Claw.h"
#include "Pivot/Pivot.h"
#include "Telescope/Telescope.h"

#include <iostream>
#include <memory>

class Arm : public ArmInterface
{
public:
    enum ARM_STATE
    {
        ARM_NONE = 0,
        ARM_IN,
        ARM_STOW,
        ARM_L3,
        ARM_L2,
        ARM_L1,
        ARM_SCORE,
        ARM_MANUAL
    };

    enum ARM_CARGO
    {
        ST_NONE = 0,
        ST_CONE,
        ST_CUBE
    };

private:
    /**
     * @brief Will rotate the arm to the specified angle
     * 
     * @param angle The desired angle in degrees to rotate to
     */
    void SetArmAngle(const double angle) override;

    /**
     * @brief Will set the telescoping position of the arm
     * 
     * @param position The desired position of the arm in inches
     */
    void SetArmPosition(const double pos) override;

    /**
     * @brief Will set wrist position
     * 
     * @param position The desired position of the wrist
    */
    void SetWristPosition(const double pos) override;

    std::shared_ptr<CowLib::CowMotorController> m_RotationMotor;
    std::shared_ptr<CowLib::CowMotorController> m_TelescopeMotor;

    CowLib::CowMotorController::PositionPercentOutput m_RotationControlRequest;

    std::unique_ptr<Telescope> m_Telescope;
    std::unique_ptr<Pivot> m_Pivot;
    std::unique_ptr<Claw> m_Claw;

    ARM_CARGO m_Cargo;
    ARM_STATE m_State;
    bool m_Orientation;

public:
    /**
     * @brief Arm Constructor
     * 
     * @param rotationMotor The id of the motor to control the rotation of the arm
     * @param telescopeMotor The id of the motor to control telescoping of the arm
     * @param wristMotor The id of the motor to control wrist movement
     * @param intakeMotor The id of the motor to control intake rollers
     * @param solenoidChannel The id of the solenoid channel for the wrist
     */
    Arm(const int rotationMotor, const int telescopeMotor, int wristMotor, int intakeMotor, int solenoidChannel);

    /**
     * @brief Default destructor
     * 
     */
    ~Arm() = default;

    /**
     * @brief sets current cargo arm is carrying
    */
    void SetArmCargo(ARM_CARGO);

    /**
     * @brief sets the arm to a given setpoint based on the state
     * does some checking for valid states
    */
    void SetArmState(ARM_STATE);

    /**
     * @brief returns current cargo of arm 
    */
    ARM_CARGO GetArmCargo();

    /**
     * @brief returns current state of arm 
    */
    ARM_STATE GetArmState();

    /**
     * @brief Update the Claw state
     * controls claw open or close and intake on or off
     */
    void UpdateClawState();

    /**
     * @brief requests update to angle of pivot
    */
    void RequestAngle(double angle);

    /**
     * @brief requests update to position of telescope
    */
    void RequestPosition(double position);

    /**
     * @brief Will reset the PID values for both rotation and telescope motors
     * with the values defined in the CowConstants
     * 
     */
    void ResetConstants() override;

    /**
     * @brief Will set the motors to their specified values
     * 
     */
    void Handle() override;

    /**
     * @brief Will set the minimum or maximum angle depending on the current position
     * Will be called periodically. 
     * 
     */
    void CheckMinMax() override;

    /**
     * @brief Will set the angle of the arm to its zero position, which is the midpoint between
     * the set minimum and maximum angles of the arm.
     * 
     * This method assumes that the min and max angles are set correctly.
     * 
     */
    void ZeroSensors() override;
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */