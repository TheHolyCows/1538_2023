#ifndef __NULL_COMMAND_H__
#define __NULL_COMMAND_H__

#include "../../CowRobot.h"
#include "RobotCommand.h"

class NullCommand : public RobotCommand {
public:
    NullCommand() = default;
    ~NullCommand() override = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __NULL_COMMAND_H__ */
