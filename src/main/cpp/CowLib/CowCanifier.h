//==================================================
// Copyright (C) 2019 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWCANIFIER_H__
#define __COWLIB_COWCANIFIER_H__

#include "../CowConstants.h"
#include "ctre/Phoenix.h"

#include <stdint.h>

namespace CowLib
{
    class CowCanifier
    {
    public:
        CowCanifier(int deviceNum);
        virtual ~CowCanifier();

        void SetLEDColor(int R, int G, int B);
        void FlashColor(int, int, int);
        void ColorWheelScroll(void);
        void Handle();

    private:
        CANifier *m_Canifier;
        int m_DeviceNum;
        int m_R;
        int m_G;
        int m_B;
        int m_FlashTimer;
        int m_ScrollIndex[3] = { 0, 0, 0 };
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
