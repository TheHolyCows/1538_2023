//==================================================
// Copyright (C) 2019 Team 1538 / The Holy Cows
//==================================================

#ifndef __AUTO_MODES_H__
#define __AUTO_MODES_H__

#include "../Controllers/AutoModeController.h"

#include <deque>
#include <map>

class AutoModes
{
private:
    static AutoModes *m_SingletonInstance;
    std::map<const char *, std::deque<RobotCommand> > m_Modes;
    std::map<const char *, std::deque<RobotCommand> >::iterator m_Iterator;

    AutoModes();

public:
    static AutoModes *GetInstance();
    std::deque<RobotCommand> GetCommandList();
    const char *GetName();
    void NextMode();
};

#endif