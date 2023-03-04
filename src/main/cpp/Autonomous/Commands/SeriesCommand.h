
#ifndef __SERIES_COMMAND_H__
#define __SERIES_COMMAND_H__

#include <deque>
#include <utility>

#include "./RobotCommand.h"

class SeriesCommand : public RobotCommand {
private:
    std::deque<RobotCommand*> m_Commands;
    RobotCommand* m_CurrentCommand;

public:
    SeriesCommand(std::deque<RobotCommand*> commands);
    ~SeriesCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __SERIES_COMMAND_H__ */
