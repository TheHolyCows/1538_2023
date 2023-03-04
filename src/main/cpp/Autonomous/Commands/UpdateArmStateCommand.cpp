#include "UpdateArmStateCommand.h"

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state)
{
    m_State = state;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_CARGO cargoType)
{
    m_CargoType = cargoType;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state, ARM_CARGO cargoType)
{
    m_State     = state;
    m_CargoType = cargoType;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state, ARM_CARGO cargoType, bool invert)
{
    m_State = state;
    m_CargoType = cargoType;
    m_Invert = true;
}

bool UpdateArmStateCommand::IsComplete()
{
    return true;
}

void UpdateArmStateCommand::Start(CowRobot *robot)
{
    ARM_STATE state = robot->GetArm()->GetArmState();
    ARM_CARGO cargo = robot->GetArm()->GetArmCargo();

    if (m_State.has_value())
    {
        state = *m_State;
    }

    if (m_CargoType.has_value())
    {
        cargo = *m_CargoType;
    }

    if (m_Invert.has_value())
    {
        cargo = *m_CargoType;
    }

    robot->SetArmState(state, cargo);
}

void UpdateArmStateCommand::Handle(CowRobot *robot)
{
    return;
}

void UpdateArmStateCommand::Finish(CowRobot *robot)
{
    return;
}