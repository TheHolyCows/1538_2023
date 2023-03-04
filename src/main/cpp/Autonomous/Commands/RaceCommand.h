#ifndef __RACE_COMMAND_H__
#define __RACE_COMMAND_H__

#include <utility>

#include "./RobotCommand.h"

class RaceCommand : public RobotCommand {
private:
    RobotCommand* m_LeadCommand;
    std::vector<RobotCommand*> m_OtherCommands;

public:
    RaceCommand(RobotCommand* leadCommand, std::vector<RobotCommand*> otherCommands);
    ~RaceCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __RACE_COMMAND_H__ */
