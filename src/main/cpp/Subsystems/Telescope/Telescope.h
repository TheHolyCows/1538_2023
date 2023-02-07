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
    void SetPosition() override;
    void RequestPosition(double pos) override;

    void ResetConstants() override;

private:
    std::shared_ptr<CowLib::CowMotorController> m_TelescopeMotor;
    CowLib::CowMotorController::PositionPercentOutput m_MotorRequest;
};