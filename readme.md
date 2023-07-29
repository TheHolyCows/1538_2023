# Team 1538 / The Holy Cows

* 33-13-0
* 2023 San Diego Regional Winners
* 2023 Newton Division Finalists

### How to use
* Make sure you have wpilib installed.
* Build the code with `./gradlew build`
* Deploy with `./gradlew deploy`
* Refresh your vscode intellisense with `./gradlew generateVSCodeConfig`
* If needed, generate a `compile_commands.json` file by running `./gradlew generateCompileCommands`

To enable desktop simulation support, you need to change `includeDesktopSupport` to `true` in `build.gradle` before
building.
It doesn't work on windows at the moment because of the `CowConstants` and `CowLogger` dependencies.

### Code Structure
The main file is `CowBase.cpp`, which creates an instance of `CowRobot` as well as both the controllers for auto and
teleop.
Depending on whether teleop or auto is enabled, the method `SetController` is called on the `CowRobot`. Every loop in
`handle`, CowRobot handle is called.

CowRobot's handle method calls each subsystem's handle method, the controller's handle method, and performs any other
tasks for robot functionality.

Subsystems control a physical subsystem of the robot and generally contain motor controllers, getters, setters, and a
handle function.

The `OperatorController` for teleop uses user inputs from the operator console to set states and values which are used
in both `CowRobot` and each subsystem.

### How to make a subsystem
The general idea for a subsystem is that it controls the lower levels of a physical section of the robot.

In this year's code, `subsystems/Arm` does not fit the typical idea of a subsystem because it contains the higher level
logic to guide the movement of the entire arm assembly. However, `subsystems/Claw`, `subsystems/Pivot`, and
`subsystems/Telescope` all follow the pattern of a typical subsystem.

Start by thinking of all the motors and pneumatics that the subsystem controls. Make a motor controller class instance
for each of them.
If they are Phoenix Pro motors, you need the associated control request structs as well.

You will probably need a get and set method for each independent motor (not followers).
These will most likely do math to convert a value understandable to a human, like an angle of the mechanism, to the
motor units (turns). For example, this year's claw subsystem has `SetIntakeSpeed`, `GetIntakeSpeed`, `GetOpen`,
`SetOpen`, `RequestWristAngle`, and `GetWristAngle`.
Sometimes, you will also need something like `GetWristSetpoint` for additional logic.

You should perform the same math operations on both the getter and the setter, just inverted.
The setter should update a setpoint variable or the control request for that motor.

The handle function should set the motor controller to that setpoint.

The constructor should initialize the motor controllers and **all** member variables. You don't want to deal with 
uninitialized doubles causing problems. It should update the neutral mode, sensor phase, and inversion for each motor 
if needed.

There should be a reset constants function that sets the PID constants for each motor.
The constructor should call `ResetConstants()`

### Control Modes
* Most mechanisms that move to (and hold) a position should use Motion Magic with PID
  * Some will use position if you don't need motion profiling
* Most basic rollers / intakes will use a simple Percent Output
* Flywheels / Shooters will probably use Velocity with PIDF
* Drivetrain will use Percent Output for maximum power or velocity for actual feedback control
  * Our swerve uses percent output calculated with `target velocity / max velocity constant`

### Swerve Drive
Our swerve drive code is broken across many classes.

Joystick inputs are sent from `OperatorController` to `SwerveDriveController`, which converts them to robot velocities
(vx, vy, omega deg/sec). And sends it to `SwerveDrive`. `SwerveDrive` uses a `CowSwerveKinematics` instance to
convert the robot velocities to individual wheel velocities. `SwerveDrive`
then sends those wheel velocities to individual `SwerveModule` instances. Each swerve module instance
handles the calculations to convert from feet per second and degrees to a motor percent output and
a position in turns.

The current `SwerveModule` wheel positions and velocities are sent back to `SwerveDrive`, which uses an instance of
`CowSwerveOdometry`, which estimates the position of the robot.

For auto, trajectories are generated using PathPlannerLib and sampled to generate target x and y values.
A holonomic controller made up of 3 PID controllers converts the trajectory state to robot velocities, which are sent
to `SwerveDrive`, which works the same as in teleop. The holonomic controller used the estimated pose from the
odometry to provide feedback control.

`CowSwerveKinematics` and `CowSwerveOdometry` rely on the WPILib equivalents. We tried to use second level kinematics
(based on [YAGSL](https://github.com/BroncBotz3481/YAGSL))
but never used the value they produced for feed forward.

### Autos
The autos are command based, with one class for each command and one file that holds all the modes (`AutoModes.cpp`).
There are utility commands such as `ParallelCommand`, `RaceCommand`, `SeriesCommand`, `WaitCommand`, and `LambdaCommand`
that can be combined for more advanced autos.

The idea is that each robot action is a command, which usually works best when most things are controlled with state
machines.

Paths for swerve are generated with PathPlanner. Each section of the auto is its own path file.
To handle the cable guard, we seperated each intake and score action into individual paths so that we could go over the
cable guard slower while not sacrificing acceleration.

At champs, our autos were somehow all off to robot right (very strange), so we split the paths into red and blue sides.
Try to avoid this if at all possible, it was a last minute change (as in mid qual matches).

### Vision
There is none. All manual.
