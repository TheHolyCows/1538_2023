#include "AutoModes.h"

#include "Commands/ClawCommand.h"
#include "Commands/LambdaCommand.h"
#include "Commands/ParallelCommand.h"
#include "Commands/PathplannerSwerveTrajectoryCommand.h"
#include "Commands/UpdateArmStateCommand.h"
#include "Commands/VisionAlignCommand.h"
#include "Commands/WaitCommand.h"

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
                             double speed       = 20.21,
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

    auto setClaw = [](ARM_CARGO type)
    {
        return new LambdaCommand(
            [type](CowRobot *bot)
            {
                bot->GetArm()->SetClawState(CLAW_INTAKE);
                bot->GetArm()->SetArmCargo(type);
                Vision::GetInstance()->SetCargo(type);
                bot->GetArm()->UpdateClawState();
                bot->GetArm()->SetClawState(CLAW_OFF);
                bot->GetArm()->UpdateClawState();
            });
    };

    auto startGroundIntake = [setClaw](ARM_CARGO cargo)
    {
        return new ParallelCommand(
            { setClaw(cargo), new UpdateArmStateCommand(ARM_GND, cargo, false), new ClawCommand(CLAW_INTAKE, 0) });
    };

    auto stow = []() {
        return new SeriesCommand({ new UpdateArmStateCommand(ARM_STOW, false), new ClawCommand(CLAW_OFF, 0) });
    };

    [[maybe_unused]] auto scoreConeL2 = []()
    {
        return new SeriesCommand({ new UpdateArmStateCommand(ARM_L2, CG_CONE, true, true),
                                   new WaitCommand(0.6, false),
                                   new ClawCommand(CLAW_EXHAUST, 0.2),
                                   new UpdateArmStateCommand(ARM_DRIVER_STOW, false),
                                   new WaitCommand(0.1, false) });
    };

    std::deque<RobotCommand *> twoPointFiveGP;

    twoPointFiveGP.push_back(new WaitCommand(0.125, false));
    twoPointFiveGP.push_back(setClaw(CG_CONE));
    twoPointFiveGP.push_back(new UpdateArmStateCommand(ARM_L2, CG_CONE, false, true));
    twoPointFiveGP.push_back(new PathplannerSwerveTrajectoryCommand("cone L2 start", 8, 4, true, true));
    twoPointFiveGP.push_back(new WaitCommand(0.6, false));
    twoPointFiveGP.push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    twoPointFiveGP.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    twoPointFiveGP.push_back(new WaitCommand(0.1, false));
    twoPointFiveGP.push_back(stow());
    twoPointFiveGP.push_back(setClaw(CG_CUBE));
    twoPointFiveGP.push_back(pathWithEvents("3 GP LZ - intake cube 1",
                                            { { 0.02, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) },
                                              { 0.15, new ClawCommand(CLAW_INTAKE, 0) },
                                              { 0.01, new UpdateArmStateCommand(ARM_GND, CG_CUBE, false) } },
                                            false,
                                            20.21,
                                            12));
    twoPointFiveGP.push_back(stow());
    twoPointFiveGP.push_back(pathWithEvents("3 GP LZ - score cube 1",
                                            { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) },
                                              { 0.1, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false, true) } },
                                            false,
                                            20.21,
                                            10));
    twoPointFiveGP.push_back(new WaitCommand(0.08, false));
    twoPointFiveGP.push_back(new ClawCommand(CLAW_EXHAUST, 0.10));
    twoPointFiveGP.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    twoPointFiveGP.push_back(new WaitCommand(0.1, false));
    twoPointFiveGP.push_back(pathWithEvents(
        "3 GP LZ - intake cube 2",
        { { 0.5, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) }, { 1, startGroundIntake(CG_CUBE) } },
        false,
        20.21,
        8));
    twoPointFiveGP.push_back(stow());

    // m_Modes["3 GP LZ ( [] [] )"] = twoPointFiveGP;
    // m_Modes["3 GP LZ ( [] [] )"].push_back(
    //     pathWithEvents("3 GP LZ - score cube 2",
    //                    { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) } },
    //                    false));
    // m_Modes["3 GP LZ ( [] [] )"].push_back(new UpdateArmStateCommand(ARM_L3, true));
    // m_Modes["3 GP LZ ( [] [] )"].push_back(new ClawCommand(CLAW_EXHAUST, 0.3));
    // m_Modes["3 GP LZ ( [] [] )"].push_back(stow());
    // m_Modes["3 GP LZ ( [] [] )"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("LZ - drive away", 20.21, 12, true, false));

    // m_Modes["2.5 GP & Balance LZ ( [] [] )"] = twoPointFiveGP;
    // m_Modes["2.5 GP & Balance LZ ( [] [] )"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true));
    // m_Modes["2.5 GP & Balance LZ ( [] [] )"].push_back(
    //     new PathplannerSwerveTrajectoryCommand("LZ - to CS outside", 10, 8, true, false));
    // m_Modes["2.5 GP & Balance LZ ( [] [] )"].push_back(new BalanceCommand(-3, 7, 15, true));

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

    std::deque<RobotCommand *> twoGPGuard;

    twoGPGuard.push_back(new WaitCommand(0.125, false));
    twoGPGuard.push_back(setClaw(CG_CONE));
    twoGPGuard.push_back(new UpdateArmStateCommand(ARM_L2, CG_CONE, false, true));
    twoGPGuard.push_back(new PathplannerSwerveTrajectoryCommand("Guard - start", 8, 4, true, true));
    twoGPGuard.push_back(new WaitCommand(0.6, false));
    twoGPGuard.push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    twoGPGuard.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    twoGPGuard.push_back(new WaitCommand(0.1, false));
    twoGPGuard.push_back(pathWithEvents("Guard - intake 1",
                                        { { 0.02, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) },
                                          { 0.15, new ClawCommand(CLAW_INTAKE, 0) },
                                          { 0.01, new UpdateArmStateCommand(ARM_GND, CG_CUBE, false) } },
                                        true,
                                        20.21,
                                        8));

    twoGPGuard.push_back(stow());

    twoGPGuard.push_back(pathWithEvents("Guard - score 1",
                                        { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) },
                                          { 0.1, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false, true) } },
                                        false,
                                        20.21,
                                        8));
    twoGPGuard.push_back(new WaitCommand(0.2, false));
    twoGPGuard.push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    twoGPGuard.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    twoGPGuard.push_back(new WaitCommand(0.2, false));
    twoGPGuard.push_back(new UpdateArmStateCommand(ARM_STOW, false));

    m_Modes["2 GP & Balance Guard ( [] )"] = twoGPGuard;
    m_Modes["2 GP & Balance Guard ( [] )"].push_back(
        new PathplannerSwerveTrajectoryCommand("Guard - to CS inside", 8, 4, true, false));
    m_Modes["2 GP & Balance Guard ( [] )"].push_back(new BalanceCommand(3, 7, 15, true));

    m_Modes["2.5 GP Guard FLIP ARM ( [] ^ )"] = twoGPGuard;
    m_Modes["2.5 GP Guard FLIP ARM ( [] ^ )"].push_back(new LambdaCommand(
        [](CowRobot *bot)
        {
            bot->GetArm()->SetClawState(CLAW_INTAKE);
            bot->GetArm()->SetArmCargo(CG_CONE);
            Vision::GetInstance()->SetCargo(CG_CONE);
            bot->GetArm()->UpdateClawState();
            bot->GetArm()->SetClawState(CLAW_OFF);
            bot->GetArm()->UpdateClawState();
        }));
    m_Modes["2.5 GP Guard FLIP ARM ( [] ^ )"].push_back(pathWithEvents(
        "Guard - intake 2",
        { { 0.15, new UpdateArmStateCommand(ARM_STOW, CG_CONE, false, false) }, { 1, startGroundIntake(CG_CONE) } },
        false,
        20.21,
        8));
    m_Modes["2.5 GP Guard FLIP ARM ( [] ^ )"].push_back(stow());

    std::deque<RobotCommand *> lzbase;
    lzbase.push_back(setClaw(CG_CONE));
    lzbase.push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, false, true));
    lzbase.push_back(new WaitCommand(1.1, false));
    lzbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.20));
    lzbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    lzbase.push_back(new WaitCommand(0.1, false));
    lzbase.push_back(stow());
    lzbase.push_back(setClaw(CG_CUBE));
    lzbase.push_back(pathWithEvents("L3 Link LZ - intake cube",
                                    { { 0.02, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) },
                                      { 0.15, new ClawCommand(CLAW_INTAKE, 0) },
                                      { 0.01, new UpdateArmStateCommand(ARM_GND, CG_CUBE, false) } },
                                    true,
                                    20.21,
                                    11));
    lzbase.push_back(stow());
    lzbase.push_back(pathWithEvents("L3 Link LZ - score cube",
                                    { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) },
                                      { 0.4, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false, true) } },
                                    false,
                                    20.21,
                                    10.25));
    lzbase.push_back(new WaitCommand(0.1, false));
    lzbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.10));
    lzbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    lzbase.push_back(new WaitCommand(0.1, false));
    lzbase.push_back(setClaw(CG_CUBE));
    lzbase.push_back(pathWithEvents(
        "L3 Link LZ - intake cone",
        { { 0.5, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) }, { 1.2, startGroundIntake(CG_CUBE) } },
        false,
        20.21,
        8.33));
    lzbase.push_back(setClaw(CG_CONE));
    lzbase.push_back(new WaitCommand(0.1, false));
    lzbase.push_back(stow());

    std::deque<RobotCommand *> bluelzbase;
    bluelzbase.push_back(setClaw(CG_CONE));
    bluelzbase.push_back(new UpdateArmStateCommand(ARM_L3, CG_CONE, false, true));
    bluelzbase.push_back(new WaitCommand(1.1, false));
    bluelzbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.20));
    bluelzbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    bluelzbase.push_back(new WaitCommand(0.1, false));
    bluelzbase.push_back(stow());
    bluelzbase.push_back(setClaw(CG_CUBE));
    bluelzbase.push_back(pathWithEvents("L3 Link LZ - intake cube BLUE",
                                        { { 0.02, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) },
                                          { 0.15, new ClawCommand(CLAW_INTAKE, 0) },
                                          { 0.01, new UpdateArmStateCommand(ARM_GND, CG_CUBE, false) } },
                                        true,
                                        20.21,
                                        11));
    bluelzbase.push_back(stow());
    bluelzbase.push_back(pathWithEvents("L3 Link LZ - score cube BLUE",
                                        { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) },
                                          { 0.4, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false, true) } },
                                        false,
                                        20.21,
                                        10.25));
    bluelzbase.push_back(new WaitCommand(0.1, false));
    bluelzbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.10));
    bluelzbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    bluelzbase.push_back(new WaitCommand(0.1, false));
    bluelzbase.push_back(setClaw(CG_CUBE));
    bluelzbase.push_back(pathWithEvents(
        "L3 Link LZ - intake cone BLUE",
        { { 0.5, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true) }, { 1.2, startGroundIntake(CG_CUBE) } },
        false,
        20.21,
        8.33));
    bluelzbase.push_back(setClaw(CG_CONE));
    bluelzbase.push_back(new WaitCommand(0.08, false));
    bluelzbase.push_back(stow());

    m_Modes["L3 Link LZ ( [] -> ^ )"] = lzbase;
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(
        pathWithEvents("L3 Link LZ - score cone",
                       { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CONE, false, true) } },
                       false));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_L3, false));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new VisionAlignCommand(0.5, CG_CONE));
    // m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new PathplannerSwerveTrajectoryCommand("slow forwards", 4, 2, true, true));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new WaitCommand(0.7, false));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new ClawCommand(CLAW_EXHAUST, 0.14));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new WaitCommand(0.1, false));
    m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));
    // m_Modes["L3 Link LZ ( [] -> ^ )"].push_back(new PathplannerSwerveTrajectoryCommand("LZ - drive away", 20.21, 16, true, false));

    m_Modes["3 GP LZ RED ( [] -> ^ )"] = lzbase;
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(
        pathWithEvents("L3 Link LZ - score cone",
                       { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CONE, false, true) } },
                       false));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_L2, false));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new VisionAlignCommand(0.4, CG_CONE));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new WaitCommand(0.5, false));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new WaitCommand(0.1, false));
    m_Modes["3 GP LZ RED ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    m_Modes["3 GP LZ BLUE ( [] -> ^ )"] = bluelzbase;
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(
        pathWithEvents("L3 Link LZ - score cone BLUE",
                       { { 0.1, new UpdateArmStateCommand(ARM_STOW, CG_CONE, false, true) } },
                       false));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_L2, false));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new VisionAlignCommand(0.4, CG_CONE));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new WaitCommand(0.5, false));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new WaitCommand(0.1, false));
    m_Modes["3 GP LZ BLUE ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, true, true));

    m_Modes["2.5 balance LZ ( [] -> ^ )"] = lzbase;
    m_Modes["2.5 balance LZ ( [] -> ^ )"].push_back(new UpdateArmStateCommand(ARM_STOW, CG_CONE, false, true));
    m_Modes["2.5 balance LZ ( [] -> ^ )"].push_back(
        new PathplannerSwerveTrajectoryCommand("LZ - to CS outside 2", 10, 8, true, false));
    m_Modes["2.5 balance LZ ( [] -> ^ )"].push_back(new BalanceCommand(-3, 7, 15, true));

    std::deque<RobotCommand *> guardbase;
    guardbase.push_back(setClaw(CG_CONE));
    guardbase.push_back(new UpdateArmStateCommand(ARM_L2, CG_CONE, false, true));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - start", 8, 8, true, true));
    guardbase.push_back(new WaitCommand(0.5, false));
    guardbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.2));
    guardbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, false));
    guardbase.push_back(new WaitCommand(0.05, false));
    guardbase.push_back(stow());
    guardbase.push_back(setClaw(CG_CUBE));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - intake 1a", 20.21, 14, false, false));
    guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - intake 1b", 20, 3, false));
    guardbase.push_back(pathWithEvents("Guard - intake 1c", { { 0.2, startGroundIntake(CG_CUBE) } }, false, 20.21, 14));
    guardbase.push_back(stow());
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - score 1a", 20.21, 14, false, true));
    guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - score 1b", 20, 3, false));
    guardbase.push_back(pathWithEvents("Guard - score 1c",
                                       { { 0.3, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false) } },
                                       false,
                                       20.21,
                                       14));
    guardbase.push_back(new VisionAlignCommand(0.3, CG_CONE));
    guardbase.push_back(new WaitCommand(0.2, false));
    guardbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.3));
    guardbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, true));
    guardbase.push_back(stow());

    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - intake 2a", 20.21, 14, false, false));
    guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - intake 2b", 20, 3, false));
    guardbase.push_back(pathWithEvents("Guard - intake 2c", { { 0.2, startGroundIntake(CG_CUBE) } }, false, 20.21, 14));
    guardbase.push_back(stow());
    guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true));
    guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - score 2a", 20.21, 14, true));
    guardbase.push_back(new UpdateArmStateCommand(ARM_L2, CG_CUBE, true, true));
    guardbase.push_back(new WaitCommand(0.2, false));
    guardbase.push_back(setClaw(CG_CONE));
    guardbase.push_back(new WaitCommand(0.1, false));
    guardbase.push_back(new ClawCommand(CLAW_NONE, 0));
    guardbase.push_back(new LambdaCommand([](CowRobot *bot) { bot->GetArm()->GetClaw().SetIntakeSpeed(-1); }));
    guardbase.push_back(new WaitCommand(0.1, false));
    guardbase.push_back(new ClawCommand(CLAW_OFF, 0));
    guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, true, true));
    // guardbase.push_back(new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, true));
    // guardbase.push_back(new PathplannerSwerveTrajectoryCommand("Guard - score 2b", 20, 3, false));
    // guardbase.push_back(pathWithEvents("Guard - score 2c",
    //                                    { { 0.3, new UpdateArmStateCommand(ARM_L2, CG_CUBE, false) } },
    //                                    false,
    //                                    20.21,
    //                                    14));
    // guardbase.push_back(new WaitCommand(0.4, false));
    // guardbase.push_back(new ClawCommand(CLAW_EXHAUST, 0.1));
    // guardbase.push_back(new UpdateArmStateCommand(ARM_DRIVER_STOW, true));
    // guardbase.push_back(stow());
    m_Modes["Guard 3 GP ( [] -> [] )"] = guardbase;

    m_Iterator = m_Modes.begin();
}

AutoModes::~AutoModes()
{
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
