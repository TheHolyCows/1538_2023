//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#pragma once

class PivotInterface
{
public:
    virtual void RequestAngle(double pos) = 0;

    virtual inline double GetAngle() { return m_Angle; }

    virtual void ResetConstants() = 0;

protected:
    double m_Angle = 0;
};