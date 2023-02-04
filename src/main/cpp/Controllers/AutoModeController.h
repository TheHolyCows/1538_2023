#ifndef __AUTONOMOUS_CONTROLLER_H__
#define __AUTONOMOUS_CONTROLLER_H__

#include "../Autonomous/Commands/RobotCommand.h"
#include "../CowLib/CowTimer.h"
#include "../CowRobot.h"
#include "./GenericController.h"

#include <iostream>
#include <string>

class AutoModeController : public GenericController
{
private:
    std::deque<RobotCommand *> m_CommandList;
    RobotCommand *m_CurrentCommand = nullptr;

    bool m_Started;

public:
    AutoModeController();
    ~AutoModeController() override;

    void SetCommandList(std::deque<RobotCommand *> commandList);

    void Start(CowRobot *bot);

    void Handle(CowRobot *bot) override;

    void Reset();
};

#endif /* __AUTONOMOUS_CONTROLLER_H__ */
