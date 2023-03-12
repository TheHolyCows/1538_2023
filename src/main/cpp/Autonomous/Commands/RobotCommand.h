#ifndef __ROBOT_COMMAND_H__
#define __ROBOT_COMMAND_H__

#include "../../CowRobot.h"

class RobotCommand {
public:
    virtual ~RobotCommand() = default;

    virtual bool IsComplete(CowRobot* robot) = 0;

    virtual void Start(CowRobot* robot) = 0;

    virtual void Handle(CowRobot* robot) = 0;

    virtual void Finish(CowRobot* robot) = 0;
};

#endif /* __ROBOT_COMMAND_H__ */
