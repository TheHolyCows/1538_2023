//==================================================
// Copyright (C) 2015 Team 1538 / The Holy Cows
//==================================================

#ifndef __UTILITY_H__
#define __UTILITY_H__

#include "../Declarations.h"

#include <algorithm>
#include <sstream>
#include <stdarg.h>

#define HZ(x) 1.0 / (double) x

namespace CowLib
{
    double LimitMix(double value, double maxValue);
    double LimitMix(double value);
    double AnalogInScale(double oldx, double center);

    void PrintToLCD(const char *format, ...);

    // converts units per robot period to units per second
    double UnitsPerSecond(double value);

    inline double Deadband(double value, double bandsize)
    {
        if (value > -bandsize && value < bandsize)
        {
            return 0;
        }
        else
        {
            return value;
        }
    }
} // namespace CowLib

#endif
