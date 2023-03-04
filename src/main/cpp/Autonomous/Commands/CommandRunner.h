#ifndef __COMMAND_RUNNER_H__
#define __COMMAND_RUNNER_H__

#include "../../CowLib/CowTimer.h"
#include "RobotCommand.h"

#include <memory>

class CommandRunner
{
public:
    enum State
    {
        NOT_STARTED,
        RUNNING,
        FINISHED
    };

    struct Timeout
    {
        bool useTimeout;
        double timeout;

        bool ShouldStop(double time) { return useTimeout && time > timeout; }
    };

private:
    RobotCommand &m_Command;

    State m_State;
    Timeout m_Timeout;

    std::unique_ptr<CowLib::CowTimer> m_Timer;

public:
    CommandRunner(RobotCommand &command, double timeout = 0);
    ~CommandRunner() = default;

    bool IsComplete(CowRobot* robot);

    void Start(CowRobot *robot);
    void Handle(CowRobot *robot);
    void Finish(CowRobot *robot);

    State GetState();
};

#endif /* __COMMAND_RUNNER_H__ */
