#include "CowBase.h"

CowBase::CowBase()
    : m_ControlBoard(new CowControlBoard()),
      m_OpController(new OperatorController(m_ControlBoard)),
      m_AutoController(new AutoModeController()),
      m_Constants(CowConstants::GetInstance())
{
    CowConstants::GetInstance()->RestoreData();
    m_Bot = new CowRobot();

    m_Display = new CowDisplay(m_Bot);

    // init logger
    CowLib::CowLogger::GetInstance();

    // SetPeriod(HZ(ROBOT_HZ));
    // GetWatchdog().SetEnabled(false);
    printf("Done constructing CowBase!\n");
}

CowBase::~CowBase()
{
    delete m_ControlBoard;
    delete m_OpController;
    delete m_AutoController;
    delete m_Display;
}

void CowBase::RobotInit()
{
    m_Bot->Reset();

    // Construct the auto modes class to load swerve trajectories
    AutoModes::GetInstance();
}

void CowBase::DisabledInit()
{
    CowConstants::GetInstance()->RestoreData();
    // m_Bot->GetGyro()->ResetAngle();
    // m_Bot->GetGyro()->BeginCalibration();
    m_Bot->Reset();
}

void CowBase::AutonomousInit()
{
    // m_Bot->GetGyro()->FinalizeCalibration();
    // m_Bot->GetGyro()->ResetAngle();

    std::cout << "Setting command list" << std::endl;
    m_AutoController->SetCommandList(AutoModes::GetInstance()->GetCommandList());
    std::cout << "Done setting command list" << std::endl;

    m_Bot->SetController(m_AutoController);
    m_Bot->Reset();

    std::cout << "Starting auto" << std::endl;
    m_AutoController->Start(m_Bot);
}

void CowBase::TeleopInit()
{
    m_Bot->StartTime();
    // m_Bot->GetGyro()->FinalizeCalibration();
    std::cout << "setting controller " << m_OpController << std::endl;
    m_Bot->SetController(m_OpController);
    std::cout << "controller set successfully" << std::endl;
    // m_Bot->GetArm()->SetBrakeMode();
}

void CowBase::DisabledPeriodic()
{
    // m_Bot->GyroHandleCalibration();

    if (m_Display)
    {
        m_Display->DisplayPeriodic();
    }

    if (m_ControlBoard->GetAutoSelectButton())
    {
        m_Constants->RestoreData();

        // TODO: change back to 7
        // if (m_ControlBoard->GetSteeringButton(7)) {
        if (m_ControlBoard->GetOperatorButton(7))
        {
            m_Bot->Reset();

            /*
             * POSITION FIRST_OWNERSHIP SECOND_OWNERSHIP DRIVE
             * iterates over AutoModes
             */
            AutoModes::GetInstance()->NextMode();
            CowLib::CowLogger::LogAutoMode(AutoModes::GetInstance()->GetName().c_str());
        }
    }
    if (m_Bot)
    {
        // m_Bot->GetArm()->DisabledCalibration();
    }

    if (m_DisabledCount++ % 5 == 0) // 50 ms tick rate
    {
        CowLib::CowLogger::LogAutoMode(AutoModes::GetInstance()->GetName().c_str());
        m_DisabledCount = 1;
    }
}

void CowBase::AutonomousPeriodic()
{
    m_Bot->handle();
}

void CowBase::TeleopPeriodic()
{
    m_Bot->handle();

    // std::cout << "gyro angle: " << m_Bot->GetGyro()->GetYaw() << std::endl;

    //    if(m_Display)
    //    {
    //        m_Display->DisplayPeriodic();
    //    }
}

int main()
{
    return frc::StartRobot<CowBase>();
}
