//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// ExponentialFilter.h
// author: JT/Cole/Constantine
// created on: 2023-1-24
//==================================================

#ifndef __COWLIB_EXPONENTIALFILTER_H__
#define __COWLIB_EXPONENTIALFILTER_H__

#include "math.h"

namespace CowLib
{

    class CowExponentialFilter
    {
    public:
        CowExponentialFilter(double Exponent);

        double Filter(double Input);
        void Reset(double Input);

    private:
        double m_Exponent;
    };

} /* namespace CowLib */

#endif /* __COWLIB_EXPONENTIALFILTER_H__ */