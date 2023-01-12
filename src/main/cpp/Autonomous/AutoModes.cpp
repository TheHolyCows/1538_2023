#include "AutoModes.h"

// TODO: make auto constants reloadable for quick testing
// TODO: still need to add timeouts

// Apparently I can just do this
AutoModes *AutoModes::s_Instance = nullptr;

AutoModes::AutoModes()
{
    // FOR THE FIRST SWERVE DRIVE ACTION IN A MODE, YOU MUST HAVE RESET ODOMETRY TRUE
    // BAD THINGS WILL HAPPEN

    // Constants for swerve trajectories are loaded in the constructor, so maybe auto modes are created in a separate LoadAutoModes function that is called in the constructor and on a button in CowBase
    // the issue with that is the change auto mode button and reset constants are the same button rn, so they would have to be split

    // m_Modes["Test"].push_back(new LambdaCommand([](CowRobot* robot) {
    //     std::cout << "Lambda command" << std::endl;
    // }));

    // Theoretically this will follow trajectory "Test1" but 3 seconds into "Test2" it will run a lambda function with full access to the CowRobot instance
    m_Modes["Test"].push_back(
        new RaceCommand(new SwerveTrajectoryCommand("Test1", 30, true, true),
                        { new SeriesCommand({
                            new WaitCommand(3, false),
                            new LambdaCommand([](CowRobot *robot) { std::cout << "Hello from lambda" << std::endl; }),
                        }) }));
    // std::cout << "Complete AutoModes constructor" << std::endl;

    m_Iterator = m_Modes.begin();
}

AutoModes::~AutoModes()
{
    // Delete everything (I hope)
    for (auto &mode : m_Modes)
    {
        for (auto command : mode.second)
        {
            delete command;
        }
    }
}

AutoModes *AutoModes::GetInstance()
{
    if (s_Instance == nullptr)
    {
        s_Instance = new AutoModes();
    }

    return s_Instance;
}

std::deque<RobotCommand *> AutoModes::GetCommandList()
{
    return m_Iterator->second;
}

std::string AutoModes::GetName()
{
    return m_Iterator->first;
}

void AutoModes::NextMode()
{
    std::cout << "Trying to get to the next move" << std::endl;

    ++m_Iterator;

    if (m_Iterator == m_Modes.end())
    {
        m_Iterator = m_Modes.begin();
    }

    // Display the name of the current auto mode to driver station
    std::string name = GetName();
    std::cout << "Auto mode: " << name << std::endl;
    // FRC_ReportError(frc::err::Error, "{}", "Auto Mode: " + name);
}
