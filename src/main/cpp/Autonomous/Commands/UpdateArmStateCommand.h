#pragma once

#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <optional>

class UpdateArmStateCommand : public RobotCommand
{
private:
    std::optional<ARM_STATE> m_State;
    std::optional<ARM_CARGO> m_CargoType;

public:
    UpdateArmStateCommand(ARM_STATE state);
    UpdateArmStateCommand(ARM_CARGO cargoType);
    UpdateArmStateCommand(ARM_STATE state, ARM_CARGO cargoType);
    ~UpdateArmStateCommand() = default;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};