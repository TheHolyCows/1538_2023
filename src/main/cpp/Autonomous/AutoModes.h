//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
//==================================================

#ifndef __AUTO_MODES_H__
#define __AUTO_MODES_H__

#include "Commands/BalanceCommand.h"
#include "Commands/ClawCommand.h"
#include "Commands/HoldPositionCommand.h"
#include "Commands/LambdaCommand.h"
#include "Commands/PathplannerSwerveTrajectoryCommand.h"
#include "Commands/RaceCommand.h"
#include "Commands/SeriesCommand.h"
#include "Commands/SwerveTrajectoryCommand.h"
#include "Commands/UpdateArmStateCommand.h"
#include "Commands/VisionAlignCommand.h"
#include "Commands/WaitCommand.h"

#include <deque>
#include <frc/Errors.h>
#include <map>
#include <string>

class AutoModes
{
private:
    AutoModes();
    ~AutoModes();
    static AutoModes *s_Instance;

    std::map<std::string, std::deque<RobotCommand *>> m_Modes;
    std::map<std::string, std::deque<RobotCommand *>>::iterator m_Iterator;

public:
    static AutoModes *GetInstance();

    std::deque<RobotCommand *> GetCommandList();

    std::string GetName();

    void NextMode();
};

#endif /* __AUTO_MODES_H__ */
