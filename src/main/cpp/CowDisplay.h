//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_DISPLAY_H__
#define __COW_DISPLAY_H__

#include "CowRobot.h"

class CowDisplay
{
private:
    void DisplayNextState();
    void DisplayUpdateState();
    void DisplayState(bool);

    uint8_t m_UserState;
    uint8_t m_PrevUserState;
    uint32_t m_UserStatePeriodicCount;
    uint32_t m_UserPeriodicCount;
    uint8_t m_UserScrollCount;
    bool m_ButtonPressedOnce;
    CowRobot *m_Bot;

public:
    CowDisplay(CowRobot *);
    void DisplayPeriodic();
};

#endif
