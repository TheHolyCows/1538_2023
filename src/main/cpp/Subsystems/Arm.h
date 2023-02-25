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

#include <algorithm>
#include <iostream>
#include <memory>

class Arm : public ArmInterface
{
private:
    /**
     * @brief Will safely set m_CurrentConfig with the specified angle
     * 
     * @param angle The angle to set the arm to
     * @param curAngle - current angle of the pivot
     * @param curExt - current extension of the telescope
     */
    double GetSafeAngle(double angle, const double curAngle, const double curExt);

    /**
     * @brief Will safely set m_CurrentConfig with the specified position
     * 
     * @param extension 
     * @param reqAngle - angle requested by the state
     * @param curExt - current extension of the telescope
     */
    double GetSafeExt(double extension, const double reqAngle, const double curExt);

    /**
     * @brief 
     * 
     */
    double GetSafeWristAngle(double curPivotAngle, double reqPivotAngle);

    // TODO: figure out if these need to exist or change them
    // Also, you can't do pure virtual functions
    void SetArmAngle(double angle) override {}

    void SetArmExtension(double ext) override {}

    std::unique_ptr<Telescope> m_Telescope;
    std::unique_ptr<Pivot> m_Pivot;
    std::unique_ptr<Claw> m_Claw;

    ARM_CARGO m_Cargo;
    ARM_STATE m_State;
    bool m_RevOrientation;

    bool m_PivotLockout;
    bool m_ExtLockout;

    int m_LoopCount;

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
     * @brief sets positive/negative values for angle and claw based on switch
    */
    void SetArmOrientation(bool value);

    /**
     * @brief Update the Claw state
     * controls claw open or close and intake on or off
     */
    void UpdateClawState();

    /**
     * @brief requests update to overall position of the arm
     * this includes the current angle of the pivot and the extension of the telescope
    */
    void RequestPosition(double angle, double extension);

    /**
     * @brief Will reset the PID values for both rotation and telescope motors
     * with the values defined in the CowConstants
     * 
     */
    void ResetConstants() override;

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
    //void ZeroSensors() override;

    /**
     * @brief Will set the motors to their specified values
     * 
     */
    void Handle() override;
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */