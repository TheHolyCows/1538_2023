//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __GENERIC_CONTROLLER_H__
#define __GENERIC_CONTROLLER_H__

class CowRobot;

class GenericController
{
public:
    virtual ~GenericController(){};
    virtual void Handle(CowRobot *bot) = 0;
};

#endif /* __GENERIC_CONTROLLER_H__ */
