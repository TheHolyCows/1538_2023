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
    virtual void SetAngle() override;
    virtual void RequestAngle(double pos) override;

    virtual void ResetConstants() override;

private:
    std::shared_ptr<CowLib::CowMotorController> m_PivotMotor;
    CowLib::CowMotorController::PositionPercentOutput m_MotorRequest;
    int m_LoopCount;
};