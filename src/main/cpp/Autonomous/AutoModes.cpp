#include "AutoModes.h"

#include "Commands/ParallelCommand.h"
#include "Commands/PathplannerSwerveTrajectoryCommand.h"
#include "Commands/UpdateArmStateCommand.h"
#include "Commands/WaitCommand.h"

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

    // // Theoretically this will follow trajectory "Test1" but 3 seconds into "Test2" it will run a lambda function with full access to the CowRobot instance
    // m_Modes["Wait 5 seconds"].push_back(new LambdaCommand([](CowRobot *robot) { robot->GetDrivetrain()->Reset(); }));
    // m_Modes["Wait 5 seconds"].push_back(new WaitCommand(10, true));
    // m_Modes["Test"].push_back(new SwerveTrajectoryCommand("output/Test1", 0, true, true));
    // m_Modes["Test"].push_back(new HoldPositionCommand(120, 0, true, false));

    m_Modes["Test"].push_back(new PathplannerSwerveTrajectoryCommand("TestA", 4, 3, true, true));

    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_L3, ST_CONE));
    // m_Modes["theoretical 2 cone"].push_back(new AprilTagAlignCommand(Vision::CONE, 2));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_SCORE));
    // m_Modes["theoretical 2 cone"].push_back(new ParallelCommand(
    //     { new PathplannerSwerveTrajectoryCommand("2ConeIntake", 5, 3, true, true),
    //       new SeriesCommand({ new WaitCommand(3, false), new UpdateArmStateCommand(ARM_IN, ST_CONE) }) }));
    // m_Modes["theoretical 2 cone"].push_back(new PathplannerSwerveTrajectoryCommand(
    //     "2ConeIntake",
    //     5,
    //     3,
    //     true,
    //     true,
    //     { PathplannerSwerveTrajectoryCommand::Event{ "start intake", new UpdateArmStateCommand(ARM_IN, ST_CONE) } }));
    // m_Modes["theoretical 2 cone"].push_back(new WaitCommand(1, false));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_STOW));
    // m_Modes["theoretical 2 cone"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("2ConeDriveToScore", 5, 3, true, false));
    // m_Modes["theoretical 2 cone"].push_back(new AprilTagAlignCommand(Vision::CONE, 2));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_L3));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_SCORE));

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
    ++m_Iterator;

    if (m_Iterator == m_Modes.end())
    {
        m_Iterator = m_Modes.begin();
    }

    // Display the name of the current auto mode to driver station
    std::string name = GetName();
    std::cout << "Auto mode: " << name << std::endl;
}
