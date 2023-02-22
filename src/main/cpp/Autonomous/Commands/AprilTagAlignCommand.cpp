#include "AprilTagAlignCommand.h"

AprilTagAlignCommand::AprilTagAlignCommand(Vision::GamePiece gamePiece, const double timeout)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_GamePiece(gamePiece),
      m_Timeout(timeout)
{
}

bool AprilTagAlignCommand::IsComplete()
{
    return Vision::GetInstance()->ScoringYAligned(m_GamePiece) && Vision::GetInstance()->ScoringYawAligned();
}

void AprilTagAlignCommand::Start(CowRobot *robot)
{
    return;
}

void AprilTagAlignCommand::Handle(CowRobot *robot)
{
    robot->GetDriveController()->AlignToScore(0, m_GamePiece);
}

void AprilTagAlignCommand::Finish(CowRobot *robot)
{
    return;
}