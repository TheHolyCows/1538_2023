#include "VisionAlignCommand.h"

VisionAlignCommand::VisionAlignCommand(const double timeout, const ARM_CARGO cargo)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Timeout(timeout),
      m_Cargo(cargo)
{
}

bool VisionAlignCommand::IsComplete(CowRobot *robot)
{
    if (m_Timer->HasElapsed(m_Timeout))
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
        return true;
    }

    // // not done if yaw not correct
    // if (m_Gyro.GetYawDegrees() < CONSTANT("HEADING_TOLERANCE"))
    // {
    //     return false;
    // }
    //
    // switch (m_Cargo)
    // {
    // case ARM_CARGO::CG_CUBE :
    //     if (Vision::GetInstance()->CubeYAligned())
    //     {
    //         return true;
    //     }
    //     break;
    // case ARM_CARGO::CG_CONE :
    //     if (Vision::GetInstance()->ConeYAligned())
    //     {
    //         return true;
    //     }
    //     break;
    // default :
    //     break;
    // }
    //
    return false;
}

void VisionAlignCommand::Start(CowRobot *robot)
{
    m_Timer->Start();
}

void VisionAlignCommand::Handle(CowRobot *robot)
{
    robot->GetDriveController()->LockHeading(0, 0);
    // switch (m_Cargo)
    // {
    // case ARM_CARGO::CG_CUBE :
    //     robot->GetDriveController()->CubeAlign(0);
    //     break;
    // case ARM_CARGO::CG_CONE :
    //     robot->GetDriveController()->ConeAlign(0, 0);
    //     break;
    // default :
    //     break;
    // }
}

void VisionAlignCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}
