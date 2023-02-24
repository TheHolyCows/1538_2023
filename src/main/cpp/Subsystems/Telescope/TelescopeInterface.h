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
    virtual void RequestPosition(double pos) = 0;

    virtual inline double GetPosition() { return m_Position; }

    virtual void ResetConstants() = 0;

protected:
    double m_Position = 0;
};