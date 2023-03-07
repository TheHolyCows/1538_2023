#include "CubeAlignCommand.h"

CubeAlignCommand::CubeAlignCommand(const double timeout)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Timeout(timeout)
{
}

bool CubeAlignCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_Timeout) || (Vision::GetInstance()->CubeYAligned() && Vision::GetInstance()->Cube3dYawAligned());
}

void CubeAlignCommand::Start(CowRobot *robot)
{
    return;
}

void CubeAlignCommand::Handle(CowRobot *robot)
{
    robot->GetDriveController()->CubeAlign(0);
}

void CubeAlignCommand::Finish(CowRobot *robot)
{
    return;
}