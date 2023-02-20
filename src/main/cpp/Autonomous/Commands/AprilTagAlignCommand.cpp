#include "AprilTagAlignCommand.h"

AprilTagAlignCommand::AprilTagAlignCommand(Vision::GamePiece gamePiece)
{
    m_GamePiece = gamePiece;
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