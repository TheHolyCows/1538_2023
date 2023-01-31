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

#include <iostream>
#include <memory>

class Arm
{
private:
    std::unique_ptr<CowLib::CowMotorController> m_RotationMotor;
    std::unique_ptr<CowLib::CowMotorController> m_TelescopeMotor;

    CowLib::CowMotorController::PositionPercentOutput m_RotationControlRequest;
    CowLib::CowMotorController::PositionPercentOutput m_TelescopeControlRequest;

    double m_TelescopePosition;
    double m_Angle;

    double m_MinAngle;
    double m_MaxAngle;

    int m_LoopCount;

public:
    /**
     * @brief Arm Constructor
     * 
     * @param rotationMotor The id of the motor to control the rotation of the arm
     * @param telescopeMotor The id of the motor to control telescoping of the arm
     */
    Arm(int rotationMotor, int telescopeMotor);

    /**
     * @brief Default destructor
     * 
     */
    ~Arm() = default;

    /**
     * @brief Will rotate the arm to the specified angle
     * 
     * @param angle The desired angle in degrees to rotate to
     */
    void SetAngle(double angle);

    /**
     * @brief Will set the telescoping position of the arm
     * 
     * @param position The desired position of the arm in inches
     */
    void SetTelescopePosition(double position);

    /**
     * @brief Will return the current angle of the arm
     * 
     * @return double The current angle of the arm in degrees 
     */
    double GetAngle() const;

    /**
     * @brief Will get the telescoping
     * 
     * @return double The desired telescoping position of the arm in inches
     */
    double GetTelescopePosition() const;

    /**
     * @brief 
     * 
     */
    void ResetConstants();

    void Handle();

    void CheckMinMax();
    void ZeroSensors();
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */