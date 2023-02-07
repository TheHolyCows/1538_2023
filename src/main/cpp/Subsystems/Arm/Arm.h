//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../../CowConstants.h"
#include "../../CowLib/Conversions.h"
#include "../../CowLib/CowMotorController.h"
#include "../Pivot/PivotInterface.h"
#include "../Telescope/TelescopeInterface.h"
#include "ArmInterface.h"

#include <iostream>
#include <memory>

class Arm : public ArmInterface
{
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

    std::shared_ptr<CowLib::CowMotorController> m_RotationMotor;
    std::shared_ptr<CowLib::CowMotorController> m_TelescopeMotor;

    CowLib::CowMotorController::PositionPercentOutput m_RotationControlRequest;

    std::unique_ptr<TelescopeInterface> m_Telescope;
    std::unique_ptr<PivotInterface> m_Pivot;

public:
    /**
     * @brief Arm Constructor
     * 
     * @param rotationMotor The id of the motor to control the rotation of the arm
     * @param telescopeMotor The id of the motor to control telescoping of the arm
     */
    Arm(const int rotationMotor, const int telescopeMotor);

    /**
     * @brief Default destructor
     * 
     */
    ~Arm() = default;

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