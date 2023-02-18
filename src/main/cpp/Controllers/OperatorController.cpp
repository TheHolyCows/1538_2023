#include "OperatorController.h"

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;

    m_EvasiveSwerveWheel = NONE;

    m_ControllerExpFilter = new CowLib::CowExponentialFilter(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));
}

void OperatorController::Handle(CowRobot *bot)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    // testing intake - remove when done
    // if (m_CB->GetLeftDriveStickButton(2))
    // {
    //     bot->SetArmState(ARM_IN, ST_CUBE);
    // }
    // if (m_CB->GetLeftDriveStickButton(4))
    // {
    //     bot->SetArmState(ARM_IN, ST_CONE);
    // }
    // if (m_CB->GetLeftDriveStickButton(5))
    // {
    //     bot->SetArmState(ARM_NONE, ST_NONE);
    // }

    if (m_CB->GetLeftDriveStickAxis(7) > 0.85)
    {
        if (m_EvasiveSwerveWheel == NONE)
        {
            double stickAngle = atan2(m_CB->GetLeftDriveStickAxis(1), m_CB->GetLeftDriveStickAxis(0)) * 180 / M_PI;
            stickAngle        = (stickAngle < 0) ? (360 + stickAngle) : stickAngle;

            double robotOrientedAngle = bot->GetGyro()->GetYawDegrees() + stickAngle;

            // set wheel based on quadrant
            if (robotOrientedAngle >= 0 && robotOrientedAngle < 90)
            {
                m_EvasiveSwerveWheel = FRONT_LEFT;
            }
            else if (robotOrientedAngle >= 90 && robotOrientedAngle < 180)
            {
                m_EvasiveSwerveWheel = FRONT_RIGHT;
            }
            else if (robotOrientedAngle >= 180 && robotOrientedAngle < 270)
            {
                m_EvasiveSwerveWheel = BACK_RIGHT;
            }
            else if (robotOrientedAngle >= 270 && robotOrientedAngle < 360)
            {
                m_EvasiveSwerveWheel = BACK_LEFT;
            }
            else
            {
                CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "Invalid angle for quick turn");
            }
        }

        const double wb = CONSTANT("WHEEL_BASE");

        switch (m_EvasiveSwerveWheel)
        {
        case FRONT_LEFT :
            centerOfRotationX = wb;
            centerOfRotationY = wb;
            break;
        case FRONT_RIGHT :
            centerOfRotationX = wb;
            centerOfRotationY = -wb;
            break;
        case BACK_LEFT :
            centerOfRotationX = -wb;
            centerOfRotationY = wb;
            break;
        case BACK_RIGHT :
            centerOfRotationX = -wb;
            centerOfRotationY = -wb;
            break;
        case NONE :
            break;
        }
    }
    else
    {
        m_EvasiveSwerveWheel = NONE;
    }

    // Left trigger
    bool fieldRelative = m_CB->GetLeftDriveStickAxis(6) < 0.85;

    // Inital Drive Values
    double x
        = m_ControllerExpFilter->Filter(CowLib::Deadband(m_CB->GetLeftDriveStickAxis(0), CONSTANT("STICK_DEADBAND")))
          * CONSTANT("DESIRED_MAX_SPEED");
    double y
        = m_ControllerExpFilter->Filter(CowLib::Deadband(m_CB->GetLeftDriveStickAxis(1), CONSTANT("STICK_DEADBAND")))
          * CONSTANT("DESIRED_MAX_SPEED");
    double omega
        = m_ControllerExpFilter->Filter(CowLib::Deadband(m_CB->GetLeftDriveStickAxis(4), CONSTANT("STICK_DEADBAND")))
          * CONSTANT("DESIRED_MAX_ANG_VEL");

    bool force = false;

    if (m_CB->GetLeftDriveStickAxis(2) > 0.85)
    {
        y             = Vision::GetInstance()->ScoringYPID(Vision::GamePiece::CONE);
        omega         = Vision::GetInstance()->ScoringYawPID();
        force         = true;
        fieldRelative = false;
    }

    // Default Drive
    bot->GetDrivetrain()->SetVelocity(x, y, omega, fieldRelative, centerOfRotationX, centerOfRotationY, force);

    // bot->ArmSM();
}

void OperatorController::ResetConstants()
{
    m_ControllerExpFilter->Reset(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));
}