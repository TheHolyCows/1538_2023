#include "AutoModeController.h"

#include "../Autonomous/Commands/NullCommand.h"

#include <utility>

AutoModeController::AutoModeController()
{
    m_Started = false;
}

// I'm pretty sure these are all pointers to the instances created in AutoModes.cpp, which handles the deletion.
// They're not deleted before the robot is turned off anyway lol
AutoModeController::~AutoModeController() = default;

// TODO: manage the memory like a smart person (not me)
void AutoModeController::SetCommandList(std::deque<RobotCommand *> commandList)
{
    // Clang-Tidy told me to use move and I don't see why not
    std::cout << "Setting command list (from controller)" << std::endl;
    m_CommandList = commandList;
}

void AutoModeController::Start(CowRobot *bot)
{
    m_CurrentCommand = m_CommandList.front();
    m_CommandList.pop_front();
    m_CurrentCommand->Start(bot);
    std::cout << "Starting command is " << m_CurrentCommand << std::endl;
    m_Started = true;
}

// 1678 does all this logic in another thread for some reason... I can't figure out why. I will not regret this.
void AutoModeController::Handle(CowRobot *bot)
{
    if (!m_Started)
    {
        return;
    }

    // std::cout << "current command: " << m_CurrentCommand << std::endl;
    // If command is nullptr, we must be done (or not started)
    if (m_CurrentCommand == nullptr)
    {
        bot->DoNothing();
        return;
    }

    // TODO: Do shared logic here

    // std::cout << "Handling command " << m_CurrentCommand << std::endl;
    m_CurrentCommand->Handle(bot);

    // If the command is done
    if (m_CurrentCommand->IsComplete(bot))
    {
        // Cleanup current command
        m_CurrentCommand->Finish(bot);

        // TODO: probably need to delete pointer here
        delete m_CurrentCommand;

        if (!m_CommandList.empty())
        {
            m_CurrentCommand = m_CommandList.front();
            m_CommandList.pop_front();

            // Setup next command
            m_CurrentCommand->Start(bot);
        }
        else
        {
            // Done
            // TODO: figure better way to stop
            m_CurrentCommand = nullptr;
        }
    }
}

// Gets called after SetCommandList btw
void AutoModeController::Reset()
{
    m_Started        = false;
    m_CurrentCommand = new NullCommand();
}
