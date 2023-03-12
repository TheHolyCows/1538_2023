
#ifndef __PARALLEL_COMMAND_H__
#define __PARALLEL_COMMAND_H__

#include <utility>

#include "./RobotCommand.h"

class ParallelCommand : public RobotCommand {
private:
    std::vector<RobotCommand*> m_Commands;

public:
    ParallelCommand(std::vector<RobotCommand*> commands);
    ~ParallelCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __PARALLEL_COMMAND_H__ */
