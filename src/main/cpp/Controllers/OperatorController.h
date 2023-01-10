//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __OPERATOR_CONTROLLER_H__
#define __OPERATOR_CONTROLLER_H__

#include "../CowConstants.h"
#include "../CowControlBoard.h"
#include "../CowLib/CowLatch.h"
#include "../CowLib/CowLib.h"
#include "../CowRobot.h"
#include "../Declarations.h"
#include "../Subsystems/Limelight.h"

#include <math.h>
#include <stdio.h>

class OperatorController : public GenericController
{
private:
    OperatorController();
    CowControlBoard *m_CB;

    enum DriverButtonMap
    {
        AUTO_AIM = 3,
    };

    enum OperatorButtonMap
    {
        SWITCH_FRONT_INTAKE  = 8,
        SWITCH_REAR_INTAKE   = 10,
        SWITCH_CLIMBER       = 12,
        BUTTON_FRONT_EXHAUST = 4,
        BUTTON_REAR_EXHAUST  = 6,
        BUTTON_FRONT_INTAKE  = 7,
        BUTTON_REAR_INTAKE   = 9,
        BUTTON_SHOOT         = 5,
        SWITCH_SHOOTER       = 3,
        BUTTON_HOOD_UP       = 1,
        BUTTON_HOOD_DOWN     = 2,
        BUTTON_HOOD_BOTTOM   = 11,
    };

public:
    OperatorController(CowControlBoard *controlboard);
    void Handle(CowRobot *bot);

    double m_TrackingCooldownTimer;
};

#endif /* __OPERATOR_CONTROLLER_H__ */
