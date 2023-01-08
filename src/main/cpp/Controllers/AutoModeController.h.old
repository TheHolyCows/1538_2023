//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __AUTO_MODE_CONTROLLER_H__
#define __AUTO_MODE_CONTROLLER_H__

#include "../CowConstants.h"
#include "../CowLib/CowTimer.h"

#include <deque>

typedef enum
{
    CMD_NULL = 0,
    CMD_TURN,
    CMD_TURN_INTAKE,
    CMD_DRIVE_DISTANCE,
    CMD_DRIVE_DISTANCE_INTAKE,
    E,
    CMD_HOLD_DISTANCE,
    CMD_HOLD_DISTANCE_INTAKE,
    CMD_VISION_ALIGN,
    CMD_INTAKE_EXHAUST,
    CMD_WAIT
} e_RobotCommand;

typedef enum
{
    INTAKE_STOP = 0,
    INTAKE_F_IN,
    INTAKE_R_IN,
    INTAKE_F_OUT,
    INTAKE_R_OUT,
    INTAKE_SHOOT,
    INTAKE_TOP_OUT
} e_IntakeMode;

class RobotCommand
{
public:
    e_RobotCommand m_Command;
    e_IntakeMode m_IntakeMode;
    double m_EncoderCount;
    double m_Heading;
    double m_Speed;
    double m_Timeout;

    RobotCommand()
        : m_Command(CMD_NULL),
          m_IntakeMode(INTAKE_STOP),
          m_EncoderCount(0),
          m_Heading(0),
          m_Speed(0),
          m_Timeout(0)
    {
    }

    RobotCommand(e_RobotCommand cmd, double encoder, double heading, double speed, double timeout)
        : m_Command(cmd),
          m_EncoderCount(encoder),
          m_Heading(heading),
          m_Speed(speed),
          m_Timeout(timeout)
    {
    }
};

#include "../CowLib/CowLib.h"
#include "../CowRobot.h"

class AutoModeController : public GenericController
{
private:
    CowLib::CowTimer *m_Timer; // TODO: standardize timing
    std::deque<RobotCommand> m_CommandList;
    RobotCommand m_CurrentCommand;

    bool m_ExhaustMode = false;

    void doNothing(CowRobot *bot);
    float m_OriginalEncoder;

public:
    AutoModeController();
    void SetCommandList(std::deque<RobotCommand> list);

    void handle(CowRobot *bot);
    void reset();
};

#endif /* __AUTO_MODE_CONTROLLER_H__ */
