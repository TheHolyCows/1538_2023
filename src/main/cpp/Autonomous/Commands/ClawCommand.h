#pragma once

#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <optional>

class ClawCommand : public RobotCommand
{
private:
    CLAW_STATE m_ClawState;
    double m_Timeout;
    bool m_Wait;

    std::unique_ptr<CowLib::CowTimer> m_Timer;

public:
    /**
     * @brief Construct a new Claw Command object. If timeout is 0, the claw will not be turned off
     *
     * @param state The state to set the claw to
     * @param timeout The amount of time to wait before turning the claw off
     */
    ClawCommand(CLAW_STATE, double timeout);
    ~ClawCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};