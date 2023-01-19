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

    // init gyro
    CowPigeon::GetInstance();

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

    CowPigeon::GetInstance()->SetYaw(0);

    m_Bot->GetDrivetrain()->Reset();
}

void CowBase::DisabledInit()
{
    CowConstants::GetInstance()->RestoreData();
    m_Bot->Reset();
}

void CowBase::AutonomousInit()
{
    // m_Bot->GetGyro()->FinalizeCalibration();
    // m_Bot->GetGyro()->ResetAngle();

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
    // m_Bot->GetArm()->SetBrakeMode(); TODO: add back in
}

void CowBase::DisabledPeriodic()
{
    // log motor info
    CowLib::CowLogger::GetInstance()->Handle();

    // m_Bot->GyroHandleCalibration();

    if (m_Display)
    {
        m_Display->DisplayPeriodic();
    }

    if (m_ControlBoard->GetLeftDriveStickButton(7))
    {
        m_Constants->RestoreData();

        // TODO: change back to 7
        // if (m_ControlBoard->GetSteeringButton(7)) {
        if (m_ControlBoard->GetLeftDriveStickButton(7))
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
        // TODO: add this back in
        // m_Bot->GetArm()->DisabledCalibration();
    }

    if (m_DisabledCount++ % 10 == 0) // 50 ms tick rate
    {
        CowLib::CowLogger::LogAutoMode(AutoModes::GetInstance()->GetName().c_str());
        // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "gyro angle %f", CowPigeon::GetInstance()->GetYaw());
        // printf("Gryo angle %f\n", CowPigeon::GetInstance()->GetYaw());
        m_DisabledCount = 1;
    }
}

void CowBase::AutonomousPeriodic()
{
    m_Bot->Handle();
}

void CowBase::TeleopPeriodic()
{
    m_Bot->Handle();
}

int main()
{
    return frc::StartRobot<CowBase>();
}
