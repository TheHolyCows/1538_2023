//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#pragma once

#include "../../CowConstants.h"
#include "../../CowLib/Conversions.h"
#include "../../CowLib/CowMotorController.h"
#include "PivotInterface.h"

#include <memory>

class Pivot : public PivotInterface
{
public:
    Pivot(const int MotorId);

    /**
     * sets variable in current position request to angle
    */
    void RequestAngle(double angle) override;

    /**
     * returns the current angle read from the motor
     * converted to degrees
    */
    double GetAngle() override;

    /**
     * update PID of pivot based on currrent arm extension
    */
    void UpdatePID(double);

    void ResetConstants() override;

    void Handle();

private:
    std::shared_ptr<CowLib::CowMotorController> m_PivotMotor;
    CowLib::CowMotorController::MotionMagicPercentOutput m_MotorRequest = { 0 };
};