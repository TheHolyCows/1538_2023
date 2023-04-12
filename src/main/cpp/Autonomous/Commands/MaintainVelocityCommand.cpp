#include "MaintainVelocityCommand.h"

#include <memory>

MaintainVelocityCommand::MaintainVelocityCommand(const double targetVelocity,
                                                 const double timeout,
                                                 const bool maintainHeading,
                                                 const bool stop,
                                                 const bool fieldOriented)
    : m_TargetVelocity(targetVelocity),
      m_Timeout(timeout),
      m_MaintainHeading(maintainHeading),
      m_FieldOriented(fieldOriented),
      m_Stop(stop),
      m_Timer(std::make_unique<CowLib::CowTimer>())
{
}

MaintainVelocityCommand::~MaintainVelocityCommand() = default;

bool MaintainVelocityCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_Timeout);
}

void MaintainVelocityCommand::Start(CowRobot *robot)
{
    // Save pose

    m_Timer->Reset();
    m_Timer->Start();
}

void MaintainVelocityCommand::Handle(CowRobot *robot)
{
    // frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();
}

void MaintainVelocityCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}
