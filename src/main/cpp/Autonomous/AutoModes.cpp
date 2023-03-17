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
    struct TrajectoryEvent
    {
        // Important: Delays are from the previous event NOT the start of the path
        double delay;
        RobotCommand *command;
    };

    auto pathWithEvents = [](const std::string &name,
                             const std::vector<TrajectoryEvent> &events,
                             bool resetOdometry = true,
                             double speed       = 16.5,
                             double accel       = 14)
    {
        std::deque<RobotCommand *> series;
        for (const auto &event : events)
        {
            series.push_back(new WaitCommand(event.delay, false));
            series.push_back(event.command);
        }

        return new ParallelCommand({ new PathplannerSwerveTrajectoryCommand(name, speed, accel, true, resetOdometry),
                                     new SeriesCommand(series) });
    };

    // High Level Piece
    auto scoreConeL3 = new SeriesCommand({ new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true),
                                           new WaitCommand(1, false),
                                           new ClawCommand(CLAW_EXHAUST, 0.2),
                                           new UpdateArmStateCommand(ARM_DRIVER_STOW, false),
                                           new WaitCommand(0.1, false),
                                           new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true) });
    auto scoreCubeL3 = new SeriesCommand({ new UpdateArmStateCommand(ARM_L3, CG_CUBE, true, true),
                                           new WaitCommand(1, false),
                                           new ClawCommand(CLAW_EXHAUST, 0.2),
                                           new UpdateArmStateCommand(ARM_DRIVER_STOW, false),
                                           new WaitCommand(0.1, false),
                                           new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true) });
    auto scoreConeL2 = new SeriesCommand({ new UpdateArmStateCommand(ARM_L2, CG_CONE, true, true),
                                           new WaitCommand(0.5, false),
                                           new ClawCommand(CLAW_EXHAUST, 0.2),
                                           new UpdateArmStateCommand(ARM_DRIVER_STOW, false),
                                           new WaitCommand(0.1, false),
                                           new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true) });
    auto scoreCubeL2 = new SeriesCommand({ new UpdateArmStateCommand(ARM_L2, CG_CUBE, true, true),
                                           new WaitCommand(0.5, false),
                                           new ClawCommand(CLAW_EXHAUST, 0.2),
                                           new UpdateArmStateCommand(ARM_DRIVER_STOW, false),
                                           new WaitCommand(0.1, false),
                                           new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true) });

    auto startGroundIntake = [](ARM_CARGO cargo) {
        return new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, cargo, false), new ClawCommand(CLAW_INTAKE, 0) });
    };

    auto stow = new SeriesCommand({ new UpdateArmStateCommand(ARM_STOW, false), new ClawCommand(CLAW_OFF, 0) });

    m_Modes["3 GP LZ"].push_back(scoreConeL2);
    m_Modes["3 GP LZ"].push_back(pathWithEvents(
        "3 GP LZ - intake cube 1",
        { { 0.5, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) }, { 1, startGroundIntake(CG_CUBE) } }));
    m_Modes["3 GP LZ"].push_back(stow);

    m_Modes["3 GP LZ"].push_back(
        pathWithEvents("3 GP LZ - score cube 1", { { 1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) } }));

    m_Modes["3 GP LZ"].push_back(scoreCubeL2);
    m_Modes["3 GP LZ"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));

    m_Modes["3 GP LZ"].push_back(pathWithEvents(
        "3 GP LZ - intake cube 2",
        { { 0.5, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) }, { 1, startGroundIntake(CG_CUBE) } }));
    m_Modes["3 GP LZ"].push_back(stow);

    m_Modes["3 GP LZ"].push_back(
        pathWithEvents("3 GP LZ - score cube 2", { { 1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) } }));
    m_Modes["3 GP LZ"].push_back(new UpdateArmStateCommand(ARM_GND, true));
    m_Modes["3 GP LZ"].push_back(new ClawCommand(CLAW_EXHAUST, 1));

    // FOR THE FIRST SWERVE DRIVE ACTION IN A MODE, YOU MUST HAVE RESET ODOMETRY TRUE
    // BAD THINGS WILL HAPPEN

    // m_Modes["test"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true));
    // m_Modes["test"].push_back(new ParallelCommand(
    //     { new PathplannerSwerveTrajectoryCommand("cone cone - grab cone (loading zone)", 16.5, 8, true, true),
    //       new SeriesCommand({ new WaitCommand(1.5, false),
    //                           new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, CG_CONE, false),
    //                                               new ClawCommand(CLAW_INTAKE, 0) }) }) }));
    // m_Modes["test"].push_back(new UpdateArmStateCommand(ARM_STOW, false));
    // m_Modes["test"].push_back(new ClawCommand(CLAW_OFF, 0));

    m_Modes["balance"].push_back(new BalanceCommand(8, 7, 100, false));
    m_Modes["balance"].push_back(new WaitCommand(1, true));
    m_Modes["balance"].push_back(new BalanceCommand(2.5, 7, 15, true));

    m_Modes["2 cone - loading zone"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));

    // Go to L3 and wait to score
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["2 cone - loading zone"].push_back(new WaitCommand(1, false));
    m_Modes["2 cone - loading zone"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));

    // Driver stow than real stow
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["2 cone - loading zone"].push_back(new WaitCommand(0.1, false));
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    // Drive and intake. Run intake 1.5 seconds in
    m_Modes["2 cone - loading zone"].push_back(new ParallelCommand(
        { new PathplannerSwerveTrajectoryCommand("cone cone - grab cone (loading zone)", 16.5, 14, true, true),
          new SeriesCommand({ new WaitCommand(1.5, false),
                              new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, CG_CUBE, false),
                                                  new ClawCommand(CLAW_INTAKE, 0) }) }) }));
    m_Modes["2 cone - loading zone"].push_back(new ClawCommand(CLAW_INTAKE, 0));
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_GND, CG_CONE, false));
    m_Modes["2 cone - loading zone"].push_back(new ClawCommand(CLAW_OFF, 0));

    // Stow and drive back
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));
    m_Modes["2 cone - loading zone"].push_back(
        new PathplannerSwerveTrajectoryCommand("cone cone - score cone (loading zone)", 16.5, 14, true, false));

    // L3 cone and score
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["2 cone - loading zone"].push_back(new WaitCommand(2, false));
    m_Modes["2 cone - loading zone"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));

    // Stow again
    m_Modes["2 cone - loading zone"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["2 cone - loading zone"].push_back(new WaitCommand(0.1, false));

    // start with cube, grab cone
    m_Modes["cube/cone - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CUBE);
            Vision::GetInstance()->SetCargo(CG_CUBE);
            bot->GetArm()->UpdateClawState();
        }));

    // Go to L3 and wait to score
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CUBE, true, true));
    m_Modes["cube/cone - loading zn"].push_back(new WaitCommand(1, false));
    m_Modes["cube/cone - loading zn"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));

    // Driver stow than real stow
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["cube/cone - loading zn"].push_back(new WaitCommand(0.1, false));
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    // Drive and intake. Run intake 1.5 seconds in
    m_Modes["cube/cone - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));
    m_Modes["cube/cone - loading zn"].push_back(new ParallelCommand(
        { new PathplannerSwerveTrajectoryCommand("cube cone - grab cone (loading zone)", 16.5, 8, true, true),
          new SeriesCommand({ new WaitCommand(1.5, false),
                              new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, CG_CONE, false),
                                                  new ClawCommand(CLAW_INTAKE, 0) }) }) }));
    m_Modes["cube/cone - loading zn"].push_back(new ClawCommand(CLAW_INTAKE, 0));
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_GND, CG_CONE, false));
    m_Modes["cube/cone - loading zn"].push_back(new ClawCommand(CLAW_OFF, 0));

    // Stow and drive back
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));
    m_Modes["cube/cone - loading zn"].push_back(
        new PathplannerSwerveTrajectoryCommand("cube cone - score cone (loading zone)", 16.5, 8, true, false));

    // L3 cone and score
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["cube/cone - loading zn"].push_back(new WaitCommand(2, false));
    m_Modes["cube/cone - loading zn"].push_back(new ClawCommand(CLAW_EXHAUST, 0.3));

    // Stow again
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["cube/cone - loading zn"].push_back(new WaitCommand(1, false));
    m_Modes["cube/cone - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    /** cone only balance mid **/

    m_Modes["1 cone/balance - mid"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));
    // Go to L3 and wait to score
    m_Modes["1 cone/balance - mid"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["1 cone/balance - mid"].push_back(new WaitCommand(1.3, false));
    m_Modes["1 cone/balance - mid"].push_back(new ClawCommand(CLAW_EXHAUST, 0.5));
    m_Modes["1 cone/balance - mid"].push_back(new WaitCommand(0.5, false));

    // Driver stow than real stow
    m_Modes["1 cone/balance - mid"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["1 cone/balance - mid"].push_back(new WaitCommand(0.1, false));
    m_Modes["1 cone/balance - mid"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    m_Modes["1 cone/balance - mid"].push_back(
        new PathplannerSwerveTrajectoryCommand("drive to charge station (middle)", 10, 5, true, true));
    m_Modes["1 cone/balance - mid"].push_back(new BalanceCommand(8, 7, 6, false));
    m_Modes["1 cone/balance - mid"].push_back(new WaitCommand(1, true));
    m_Modes["1 cone/balance - mid"].push_back(new BalanceCommand(2.5, 7, 4, true));

    /** cube only balance mid **/

    m_Modes["1 cube/balance - mid"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CUBE);
            Vision::GetInstance()->SetCargo(CG_CUBE);
            bot->GetArm()->UpdateClawState();
        }));
    // Go to L3 and wait to score
    m_Modes["1 cube/balance - mid"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CUBE, true, true));
    m_Modes["1 cube/balance - mid"].push_back(new WaitCommand(1.3, false));
    m_Modes["1 cube/balance - mid"].push_back(new ClawCommand(CLAW_EXHAUST, 0.5));
    m_Modes["1 cube/balance - mid"].push_back(new WaitCommand(0.5, false));

    // Driver stow than real stow
    m_Modes["1 cube/balance - mid"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["1 cube/balance - mid"].push_back(new WaitCommand(0.1, false));
    m_Modes["1 cube/balance - mid"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true));

    m_Modes["1 cube/balance - mid"].push_back(
        new PathplannerSwerveTrajectoryCommand("drive to charge station (middle cube)", 10, 5, true, true));
    m_Modes["1 cube/balance - mid"].push_back(new BalanceCommand(8, 7, 6, false));
    m_Modes["1 cube/balance - mid"].push_back(new WaitCommand(1, true));
    m_Modes["1 cube/balance - mid"].push_back(new BalanceCommand(2.5, 7, 4, true));

    /** 1.5 cone + grab cube + balance **/

    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));

    // Go to L3 and wait to score
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new WaitCommand(1, false));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));

    // Driver stow than real stow
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new WaitCommand(0.1, false));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true));

    // Drive and intake. Run intake 1.5 seconds in
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CUBE);
            Vision::GetInstance()->SetCargo(CG_CUBE);
            bot->GetArm()->UpdateClawState();
        }));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new ParallelCommand(
        { new PathplannerSwerveTrajectoryCommand("drive to piece (loading zone side)", 16.5, 8, true, true),
          new SeriesCommand({ new WaitCommand(1.5, false),
                              new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, CG_CUBE, false),
                                                  new ClawCommand(CLAW_INTAKE, 0) }) }) }));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new ClawCommand(CLAW_INTAKE, 0));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_GND, CG_CUBE, false));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new ClawCommand(CLAW_OFF, 0));

    // Stow and drive to charge station
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(
        new PathplannerSwerveTrajectoryCommand("drive to charge (loading zone)", 16.5, 10, true, false));
    m_Modes["1.5 cone/cube/balance - loading zn"].push_back(new BalanceCommand(-2.5, 7, 5, true));

    /** 1.5 cone only + balance **/

    m_Modes["1.5 cone only/balance - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));

    // Go to L3 and wait to score
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, true, true));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new WaitCommand(1, false));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));

    // Driver stow than real stow
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new WaitCommand(0.1, false));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    // Drive and intake. Run intake 1.5 seconds in
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new ParallelCommand(
        { new PathplannerSwerveTrajectoryCommand("drive to piece (loading zone side)", 16.5, 8, true, true),
          new SeriesCommand({ new WaitCommand(1.5, false),
                              new SeriesCommand({ new UpdateArmStateCommand(ARM_GND, CG_CONE, false),
                                                  new ClawCommand(CLAW_INTAKE, 0) }) }) }));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new ClawCommand(CLAW_INTAKE, 0));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_GND, CG_CONE, false));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new ClawCommand(CLAW_OFF, 0));

    // Stow and drive to charge station
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(
        new PathplannerSwerveTrajectoryCommand("drive to charge (loading zone)", 16.5, 10, true, false));
    // m_Modes["1.5 cone only/balance - loading zn"].push_back(new BalanceCommand(-8, 7, 6, false));
    // m_Modes["1.5 cone only/balance - loading zn"].push_back(new WaitCommand(1, true));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new BalanceCommand(-2.5, 7, 5, true));
    m_Modes["1.5 cone only/balance - loading zn"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
        }));

    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("c", 5, 3, true, true));
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back();
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(new WaitCommand(1, false));
    //     new PathplannerSwerveTrajectoryCommand("intake (loading zone side)", 2, 1, true, false));
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(new ClawCommand(CLAW_OFF, 0));
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("drive to balance (loading zone side)", 5, 3, true, false));
    // m_Modes["1 cone + get cube + balance (loading zone size)"].push_back(new BalanceCommand(1, 3, 3));

    //    m_Modes["Test"].push_back(new WaitCommand(, false));
    //    m_Modes["Test"].push_back(new LambdaCommand([](CowRobot *bot) { bot->GetArm()->InvertArm(true); }));
    //    m_Modes["Test"].push_back(new UpdateArmStateCommand(ARM_GND, CG_CONE, true));
    //    m_Modes["Test"].push_back(new WaitCommand(2, false));
    // //    m_Modes["Test"].push_back(new UpdateArmStateCommand(ARM_IN));
    //    m_Modes["Test"].push_back(new WaitCommand(2, false));
    //    m_Modes["Test"].push_back(new UpdateArmStateCommand(ARM_STOW));

    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE));
    // m_Modes["theoretical 2 cone"].push_back(new AprilTagAlignCommand(Vision::CONE, 2));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_SCORE));
    // m_Modes["theoretical 2 cone"].push_back(new ParallelCommand(
    //     { new PathplannerSwerveTrajectoryCommand("2ConeIntake", 5, 3, true, true),
    //       new SeriesCommand({ new WaitCommand(3, false), new UpdateArmStateCommand(ARM_IN, CG_CONE) }) }));
    // m_Modes["theoretical 2 cone"].push_back(new PathplannerSwerveTrajectoryCommand(
    //     "2ConeIntake",
    //     5,
    //     3,
    //     true,
    //     true,
    //     { PathplannerSwerveTrajectoryCommand::TrajectoryEvent{ "start intake", new UpdateArmStateCommand(ARM_IN, CG_CONE) } }));
    // m_Modes["theoretical 2 cone"].push_back(new WaitCommand(1, false));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_STOW));
    // m_Modes["theoretical 2 cone"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("2ConeDriveToScore", 5, 3, true, false));
    // m_Modes["theoretical 2 cone"].push_back(new AprilTagAlignCommand(Vision::CONE, 2));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_L3));
    // m_Modes["theoretical 2 cone"].push_back(new UpdateArmStateCommand(ARM_SCORE));
    //
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
