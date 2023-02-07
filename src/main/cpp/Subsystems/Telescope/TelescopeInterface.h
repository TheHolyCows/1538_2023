//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#pragma once

class TelescopeInterface
{
public:
    virtual void SetPosition()               = 0;
    virtual void RequestPosition(double pos) = 0;

    inline double GetPosition() const { return m_Position; }

    virtual void ResetConstants() = 0;

protected:
    double m_Position;
};