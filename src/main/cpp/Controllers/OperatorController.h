//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __OPERATOR_CONTROLLER_H__
#define __OPERATOR_CONTROLLER_H__

#include "../ControlBoards/GenericControlBoard.h"
#include "../CowConstants.h"
#include "../CowLib/CowExponentialFilter.h"
#include "../CowLib/CowLatch.h"
#include "../CowLib/CowLib.h"
#include "../CowRobot.h"
#include "../Declarations.h"
#include "../Subsystems/ArmState.h"
#include "../Subsystems/Limelight.h"
#include "../Subsystems/Vision.h"
#include "frc/controller/PIDController.h"

#include <iostream>
#include <math.h>
#include <stdio.h>

class OperatorController : public GenericController
{
private:
    OperatorController();
    GenericControlBoard *m_CB;

    enum DriverButtonMap
    {
    };

    enum OperatorButtonMap
    {
    };

    // enum Wheel
    // {
    //     NONE = -1,
    //     FRONT_LEFT,
    //     FRONT_RIGHT,
    //     BACK_LEFT,
    //     BACK_RIGHT
    // };

    // Wheel m_EvasiveSwerveWheel;

public:
    OperatorController(GenericControlBoard *controlboard);
    void Handle(CowRobot *bot);

    double m_TrackingCooldownTimer;
};

#endif /* __OPERATOR_CONTROLLER_H__ */
