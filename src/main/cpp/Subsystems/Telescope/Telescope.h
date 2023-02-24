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
#include "TelescopeInterface.h"

#include <memory>

class Telescope : public TelescopeInterface
{
public:
    Telescope(const int MotorId);

    /**
     * sets variable in current position request to pos
    */
    void RequestPosition(double pos) override;

    /**
     * returns the current encoder read from the motor
     * not currently converted
    */
    double GetPosition() override;

    /**
     * update PID of pivot based on currrent arm extension
    */
    void UpdatePID(double);

    void ResetConstants() override;

    void Handle();

private:
    std::shared_ptr<CowLib::CowMotorController> m_TelescopeMotor;
    CowLib::CowMotorController::PositionPercentOutput m_MotorRequest = { 0 };
};