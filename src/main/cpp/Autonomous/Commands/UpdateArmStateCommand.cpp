#include "UpdateArmStateCommand.h"

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state, bool waitForCompletion)
{
    m_State             = state;
    m_WaitForCompletion = waitForCompletion;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_CARGO cargoType, bool waitForCompletion)
{
    m_CargoType         = cargoType;
    m_WaitForCompletion = waitForCompletion;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state, ARM_CARGO cargoType, bool waitForCompletion)
{
    m_State             = state;
    m_CargoType         = cargoType;
    m_WaitForCompletion = waitForCompletion;
}

UpdateArmStateCommand::UpdateArmStateCommand(ARM_STATE state, ARM_CARGO cargoType, bool waitForCompletion, bool invert)
{
    m_State             = state;
    m_CargoType         = cargoType;
    m_WaitForCompletion = waitForCompletion;
    m_Invert            = invert;
}

bool UpdateArmStateCommand::IsComplete(CowRobot *robot)
{
    if (!m_WaitForCompletion)
    {
        return true;
    }

    return robot->GetArm()->AtTarget();
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
        robot->GetArm()->InvertArm(*m_Invert);
    }

    robot->GetArm()->SetArmCargo(cargo);
    robot->SetArmState(state, cargo);
    robot->GetArm()->UpdateClawState();
}

void UpdateArmStateCommand::Handle(CowRobot *robot)
{
    return;
}

void UpdateArmStateCommand::Finish(CowRobot *robot)
{
    return;
}