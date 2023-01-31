#include "ChargeStationBalanceCommand.h"

#include "frc/controller/PIDController.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "units/acceleration.h"
#include "units/velocity.h"

ChargeStationBalanceCommand::ChargeStationBalanceCommand(double timeout)
{
    m_Timeout = timeout;

    m_PIDController = new frc2::PIDController{
        CONSTANT("BALANCE_P"),
        CONSTANT("BALANCE_I"),
        CONSTANT("BALANCE_D"),
    };

    m_PIDController->SetTolerance(CONSTANT("BALANCE_PITCH_TOLERANCE"));

    m_Timer = new CowLib::CowTimer();
}

ChargeStationBalanceCommand::~ChargeStationBalanceCommand()
{
}

bool ChargeStationBalanceCommand::IsComplete()
{
    return m_Timer->HasElapsed(m_Timeout);
}

void ChargeStationBalanceCommand::Start(CowRobot *robot)
{
    m_Timer->Start();
}

void ChargeStationBalanceCommand::Handle(CowRobot *robot)
{
    // drive until pitch is changing
    if (!m_StartedDrivingUp)
    {
        if (fabs(CowPigeon::GetInstance()->GetPitchDegrees()) > CONSTANT("BALANCE_PITCH_THRESHOLD"))
        {
            m_StartedDrivingUp = true;
        }
        else
        {
            // pid pitch to 0
            auto output = -m_PIDController->Calculate(CowPigeon::GetInstance()->GetPitchDegrees(), 0.0);

            robot->GetDrivetrain()->SetVelocity(output, 0, 0);
        }
    }
}

void ChargeStationBalanceCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}