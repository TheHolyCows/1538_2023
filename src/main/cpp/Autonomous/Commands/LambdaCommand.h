#ifndef __LAMBDA_COMMAND_H__
#define __LAMBDA_COMMAND_H__

#include <functional>
#include <utility>
#include <iostream>

#include "../../CowRobot.h"
#include "RobotCommand.h"

class LambdaCommand : public RobotCommand {
private:
    std::function<void(CowRobot*)> m_Lambda;

    double m_TimeToWait;

public:
    explicit LambdaCommand(std::function<void(CowRobot*)> lambda);
    ~LambdaCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __LAMBDA_COMMAND_H__ */
