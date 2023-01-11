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
    };

    enum OperatorButtonMap
    {
    };

public:
    OperatorController(CowControlBoard *controlboard);
    void Handle(CowRobot *bot);

    double m_TrackingCooldownTimer;
};

#endif /* __OPERATOR_CONTROLLER_H__ */
