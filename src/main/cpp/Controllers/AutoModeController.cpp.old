#include "AutoModeController.h"

#include <iostream>

AutoModeController::AutoModeController()
    : m_Timer(new CowLib::CowTimer()),
      m_CurrentCommand(RobotCommand())
{
    m_Timer->Start();
    reset();

    m_OriginalEncoder = 0;
}

void AutoModeController::SetCommandList(std::deque<RobotCommand> list)
{
    m_CommandList = list;
}

void AutoModeController::reset()
{
    CowConstants::GetInstance();

    m_CommandList.clear();
    m_CurrentCommand = RobotCommand();
}

void AutoModeController::handle(CowRobot *bot)
{
    bool result = false;

    // Run the command
    switch (m_CurrentCommand.m_Command)
    {
    case CMD_NULL :
    {
        doNothing(bot);
        result = true;
        break;
    }
    case CMD_WAIT :
    {
        bot->m_Drivetrain->pidHeadingDrive(m_CurrentCommand.m_Heading, 0);
        doNothing(bot);
        break;
    }
    case CMD_TURN :
    {
        result = bot->TurnToHeading(m_CurrentCommand.m_Heading);
        break;
    }
    case CMD_TURN_INTAKE : // Why does this exist?
    {
        result = bot->TurnToHeading(m_CurrentCommand.m_Heading);
        break;
    }
    case CMD_VISION_ALIGN :
    {
        // bot->DoVisionTracking(0, CONSTANT("TRACKING_THRESHOLD"));
        break;
    }
    case CMD_HOLD_DISTANCE :
    {
        bot->m_Drivetrain->pidDistanceHeadingDrive(m_CurrentCommand.m_EncoderCount, m_CurrentCommand.m_Heading, m_CurrentCommand.m_Speed);
        result = false;
        break;
    }
    case CMD_HOLD_DISTANCE_INTAKE :
    {
        bot->m_Drivetrain->pidDistanceHeadingDrive(m_CurrentCommand.m_EncoderCount, m_CurrentCommand.m_Heading, m_CurrentCommand.m_Speed);
        result = false;
        break;
    }
    case CMD_DRIVE_DISTANCE :
    {
        // direction is forward or backwards
        float direction = 1;
        // std::cout << "OriginalEncoder: " << m_OriginalEncoder << "  EncoderCount: " << m_CurrentCommand.m_EncoderCount << std::endl;
        if (m_OriginalEncoder > m_CurrentCommand.m_EncoderCount)
        {
            // We want to go backward
            direction = -1;
        }

        bot->m_Drivetrain->pidHeadingDrive(m_CurrentCommand.m_Heading, m_CurrentCommand.m_Speed * direction);

        if (direction == 1) // Going forward
        {
            if (bot->GetDriveDistance() > m_CurrentCommand.m_EncoderCount)
            {
                result = true;
            }
        }
        else // Going backward
        {
            if (bot->GetDriveDistance() < m_CurrentCommand.m_EncoderCount)
            {
                result = true;
            }
        }

        break;
    }
    case CMD_DRIVE_DISTANCE_INTAKE :
    {
        float direction = 1;
        if (m_OriginalEncoder > m_CurrentCommand.m_EncoderCount)
        {
            // We want to go backward
            direction = -1;
        }

        bot->m_Drivetrain->pidHeadingDrive(m_CurrentCommand.m_Heading, m_CurrentCommand.m_Speed * direction);

        if (direction == 1) // Going forward
        {
            if (bot->GetDriveDistance() > m_CurrentCommand.m_EncoderCount)
            {
                result = true;
            }
        }
        else // Going backward
        {
            if (bot->GetDriveDistance() < m_CurrentCommand.m_EncoderCount)
            {
                result = true;
            }
        }
        break;
    }
    case CMD_INTAKE_EXHAUST :
    {
        doNothing(bot);
        // bot->ResetEncoders(); - don't understand the reason for this
        break;
    }
    default :
    {
        doNothing(bot);
        result = true;
        break;
    }
    }

    // Check if this command is done / .value() because of seconds_t
    if (result == true || m_CurrentCommand.m_Command == CMD_NULL || m_Timer->Get() > m_CurrentCommand.m_Timeout)
    {
        // This command is done, go get the next one
        if (m_CommandList.size() > 0)
        {
            if (m_CurrentCommand.m_Command == CMD_TURN)
            {
                // bot->ResetEncoders(); - dont understand the reason for this
            }
            m_CurrentCommand  = m_CommandList.front();
            m_OriginalEncoder = bot->GetDriveDistance();
            m_CommandList.pop_front();
            // bot->GetEncoder()->Reset(); - this is pretty old code i'd imagine

            if (!m_CurrentCommand.m_Command == CMD_NULL)
            {
                printf("Time elapsed: %f\n", m_Timer->Get());
            }
        }
        else
        {
            // We're done clean up
            m_CurrentCommand = RobotCommand();
        }
        m_Timer->Reset();
    }
}

// Drive Functions
void AutoModeController::doNothing(CowRobot *bot)
{
    bot->DriveLeftRight(0, 0);
}
