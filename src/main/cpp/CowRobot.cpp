#include "CowRobot.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    // uncomment for b-bot
    m_PowerDistributionPanel = new frc::PowerDistribution(1, frc::PowerDistribution::ModuleType::kRev);
    // m_PowerDistributionPanel = new frc::PowerDistribution();

    // mxp board was removed from robot - can remove this code
    m_LEDDisplay = nullptr;

    m_Gyro = CowPigeon::GetInstance();

    m_PreviousGyroError = 0;
    // m_Gyro->Reset(); - don't know why we have this commented
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::Accelerometer::kRange_4G);

    // Set up drivetrain
    // TODO: reset constants needs to reset this
    // fl, fr, bl, br
    // drive motor, angle motor, encoder canId's
    SwerveDrive::ModuleConstants swerveModuleConstants[4]{
        SwerveDrive::ModuleConstants{ 6, 5, 27, CONSTANT("SWERVE_FL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 8, 7, 28, CONSTANT("SWERVE_FR_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 4, 3, 26, CONSTANT("SWERVE_BL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 2, 1, 25, CONSTANT("SWERVE_BR_ENCODER_OFFSET") }
    };
    //    4, 3, 26, CONSTANT("SWERVE_FL_ENCODER_OFFSET") },
    //    SwerveDrive::ModuleConstants{ 6, 5, 27, CONSTANT("SWERVE_FR_ENCODER_OFFSET") },
    //    SwerveDrive::ModuleConstants{ 2, 1, 25, CONSTANT("SWERVE_BL_ENCODER_OFFSET") },
    //    SwerveDrive::ModuleConstants{ 8, 7, 28, BR

    m_Drivetrain = new SwerveDrive(swerveModuleConstants, CONSTANT("WHEEL_BASE"));

    m_Drivetrain->ResetEncoders();

    m_DriveController = new SwerveDriveController(*m_Drivetrain);

    m_Arm = new Arm(9, 10, 11, 12, 4);

    m_PrevArmState = ARM_NONE;

    m_Arm->SetArmCargo(CG_CONE);
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_PreviousGyroError = 0;

    m_Drivetrain->ResetConstants();
    m_DriveController->ResetConstants();
    m_Arm->ResetConstants();
    // m_Controller->ResetConstants(); TODO: error

    Vision::GetInstance()->Reset();

    CowLib::CowLogger::GetInstance()->Reset();
}

/**
 * @brief
 *
 * @param controller
 */
void CowRobot::SetController(GenericController *controller)
{
    m_Controller = controller;
}

void CowRobot::PrintToDS()
{
    if (m_DSUpdateCount++ % 10 == 0)
    {
        m_DSUpdateCount = 1;
    }
}

// Used to handle the recurring logic funtions inside the robot.
// Please call this once per update cycle.
void CowRobot::Handle()
{
    m_MatchTime = CowLib::CowTimer::GetFPGATimestamp() - m_StartTime;

    if (m_Controller == nullptr)
    {
        printf("No controller for CowRobot!!\n");
        return;
    }

    ArmSM();

    m_Controller->Handle(this);
    m_Drivetrain->Handle();
    m_Arm->Handle();

    // logger code below should have checks for debug mode before sending out data
    CowLib::CowLogger::GetInstance()->Handle();
    // log the following every 200 ms
    // if (m_DSUpdateCount % 15 == 0)
    // {
    //     // m_DSUpdateCount is reset in PrintToDS
    //     CowLib::CowLogger::LogGyro(m_Gyro);
    //     CowLib::CowLogger::LogPose(m_Drivetrain->GetPoseX(), m_Drivetrain->GetPoseY(), m_Drivetrain->GetPoseRot());
    // }

    //    // APRIL TAG BOTPOSE
    //    std::optional<Vision::BotPoseResult> visionPose = Vision::GetInstance()->GetBotPose();
    //    if (visionPose.has_value())
    //    {
    //        m_Drivetrain->AddVisionMeasurement((*visionPose).pose, (*visionPose).timestamp);
    //    }

    // accelerometers
    // double zVal = m_ZFilter.Calculate(m_Accelerometer->GetZ());

    // positive is true, negative is false
    // bool direction = (zVal - m_PrevZ) > 0 ? true : false;
    // m_PrevZ = zVal;

    PrintToDS();

    // double xAccel = m_Accelerometer->GetX();
    // // frc::SmartDashboard::PutNumber("x accel", xAccel);
    // // frc::SmartDashboard::PutNumber("gyro yaw", m_Gyro->GetYawDegrees());
    // if (m_Arm->GetArmState() == ARM_HUMAN && m_Arm->GetClawState() == CLAW_INTAKE
    //     && fabs(xAccel) > CONSTANT("GYRO_RESET_ACCEL"))
    // {
    //     m_Gyro->SetYaw(0);
    //     m_DriveController->ResetHeadingLock();
    // }
}

void CowRobot::StartTime()
{
    m_StartTime = CowLib::CowTimer::GetFPGATimestamp();
}

void CowRobot::DoNothing()
{
    // TODO: make the robot stop (including drive)
}

/**
 * @brief enables automatic stow when intaking cargo
 * only called by OperatorController when intaking
*/
void CowRobot::AllowAutoStow()
{
    if (m_Arm->GetArmState() == ARM_GND)
    {
        if (!m_AutoStowAllowed)
        {
            m_Arm->GetClaw().ResetStowTimer();
        }
        m_AutoStowAllowed = true;
    }
    else
    {
        m_AutoStowAllowed = false;
    }
}

/**
 * @brief Updates arm state based on inputs from operator
 * 
 */
void CowRobot::SetArmState(ARM_STATE state, ARM_CARGO cargo)
{
    m_Arm->UseManualControl(false);

    if (state != m_Arm->GetArmState())
    {
        m_PrevArmState = m_Arm->GetArmState();
    }
    // set to driver stow if stow is pressed after scoring or if currently in DRIVER_STOW and button is hit
    if (state == ARM_DRIVER_STOW
        && (m_PrevArmState == ARM_L2 || m_PrevArmState == ARM_L3 || m_Arm->GetArmState() == ARM_DRIVER_STOW))
    {
        m_Arm->SetArmState(state);
    }
    else if (state == ARM_DRIVER_STOW) // otherwise set to standard stow
    {
        m_Arm->SetArmState(ARM_STOW);
    }
    else
    {
        m_Arm->SetArmState(state);
    }

    //    if (state == ARM_IN)
    //    {
    //        m_Arm->SetArmCargo(cargo);
    //    }
}

/**
 * called each cycle by operator controller (at the bottom)
*/
void CowRobot::ArmSM()
{
    switch (m_Arm->GetArmState())
    {
    case ARM_NONE :
        // this state is only reachable from toggling intake
        // should only turn intake off?
        m_Arm->UpdateClawState();
        break;
    case ARM_STOW :
        m_Arm->UpdateClawState();
        m_Arm->RequestPosition(CONSTANT("ARM_STOW_ANGLE"), CONSTANT("ARM_STOW_EXT"));
        break;
    case ARM_L3 :
        if (m_Arm->GetArmCargo() == CG_CUBE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_L3_CUBE_ANGLE"),
                                   CONSTANT("ARM_L3_CUBE_EXT"),
                                   CONSTANT("WRIST_OFFSET_L3_CUBE"));
        }
        else if (m_Arm->GetArmCargo() == CG_CONE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_L3_CONE_ANGLE"),
                                   CONSTANT("ARM_L3_CONE_EXT"),
                                   CONSTANT("WRIST_OFFSET_SCORE_CONE"));
        }
        break;
    case ARM_L2 :
        if (m_Arm->GetArmCargo() == CG_CUBE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_L2_CUBE_ANGLE"),
                                   CONSTANT("ARM_L2_CUBE_EXT"),
                                   CONSTANT("WRIST_OFFSET_L2_CUBE"));
        }
        else if (m_Arm->GetArmCargo() == CG_CONE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_L2_CONE_ANGLE"),
                                   CONSTANT("ARM_L2_CONE_EXT"),
                                   CONSTANT("WRIST_OFFSET_SCORE_CONE"));
        }
        break;
    case ARM_GND :
        // auto stow check - should be called every cycle in ARM_GND
        if (m_AutoStowAllowed && m_Arm->GetClaw().IsStalled())
        {
            SetArmState(ARM_STOW, CG_NONE);
            m_AutoStowAllowed = false;
            break;
        }

        if (m_Arm->GetArmCargo() == CG_CUBE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_GND_CUBE_ANGLE"),
                                   CONSTANT("ARM_GND_CUBE_EXT"),
                                   CONSTANT("WRIST_OFFSET_IN_CUBE"));
        }
        else
        {
            m_Arm->RequestPosition(CONSTANT("ARM_GND_CONE_ANGLE"),
                                   CONSTANT("ARM_GND_CONE_EXT"),
                                   CONSTANT("WRIST_OFFSET_IN_CONE"));
        }
        break;
    case ARM_HUMAN :
        if (m_Arm->GetArmCargo() == CG_CUBE)
        {
            m_Arm->RequestPosition(CONSTANT("ARM_HUM_CUBE_ANGLE"),
                                   CONSTANT("ARM_HUM_CUBE_EXT"),
                                   CONSTANT("WRIST_OFFSET_HUM_CUBE"));
        }
        else
        {
            m_Arm->RequestPosition(CONSTANT("ARM_HUM_CONE_ANGLE"),
                                   CONSTANT("ARM_HUM_CONE_EXT"),
                                   CONSTANT("WRIST_OFFSET_HUM_CONE"));
        }
        break;
    case ARM_DRIVER_STOW :
        m_Arm->RequestSafeStow();
        break;
    case ARM_UP :
        m_Arm->RequestPosition(0, 0);
    default :
        break;
    }
}
