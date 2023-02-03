#include "SwerveModule.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/kinematics/SwerveModuleState.h>

/**
 * @brief Construct a new SwerveModule object
 * @param id Module ID
 * @param driveMotor Drive motor ID
 * @param rotationMotor Rotation motor ID
 * @param encoderId CANCoder ID
 * @param encoderOffset Absolute encoder offset
 * @param locationX Module X translation
 * @param locationY Module Y translation
 */
SwerveModule::SwerveModule(int id, int driveMotor, int rotationMotor, int encoderId, double encoderOffset)
{
    m_Id = id;

    m_DriveMotor    = new CowLib::CowMotorController(driveMotor);
    m_RotationMotor = new CowLib::CowMotorController(rotationMotor);
    m_Encoder       = new CowLib::CowCANCoder(encoderId);

    auto driveConfig = ctre::phoenixpro::configs::TalonFXConfiguration{};
    m_DriveMotor->ApplyConfig(driveConfig);
    m_DriveControlRequest = { 0 };

    auto rotationConfig = ctre::phoenixpro::configs::TalonFXConfiguration{};

    m_RotationMotor->ApplyConfig(rotationConfig);
    m_RotationMotor->SetInverted(true);
    m_RotationControlRequest = { 0 };

    // init stat
    m_Velocity      = 0;
    m_Angle         = 0;
    m_Position      = 0;
    m_PreviousAngle = 0;

    m_EncoderOffset = encoderOffset;

    ResetConstants();
    ResetEncoders();

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                              "Module %d abs encoder angle: %f  motor angle %f  \n",
                              id,
                              m_Encoder->GetAbsolutePosition(),
                              m_RotationMotor->GetPosition());

    // m_DriveMotor->GetInternalTalon()->GetSimState().SetSupplyVoltage(12_V);
    // m_RotationMotor->GetInternalTalon()->GetSimState().SetSupplyVoltage(12_V);
}

SwerveModule::~SwerveModule()
{
    delete m_DriveMotor;
    delete m_RotationMotor;
    delete m_Encoder;
    // delete m_DriveControlRequest;
    // delete m_RotationControlRequest;
}

/**
 * @brief Returns the current module state
 * @return Physical module state
 */
CowLib::CowSwerveModuleState SwerveModule::GetState()
{
    return CowLib::CowSwerveModuleState{ m_Velocity, m_Angle };
}

CowLib::CowSwerveModulePosition SwerveModule::GetPosition()
{
    return CowLib::CowSwerveModulePosition{ m_Position, m_Angle };
}

/**
 * @brief Sets the desired module state
 * @param state Target state
 */
void SwerveModule::SetTargetState(CowLib::CowSwerveModuleState state)
{
    // CowLib::CowSwerveModuleState optimized = Optimize(state, m_Angle);
    auto optimized = state;
    // auto wpistate
    //     = frc::SwerveModuleState{ units::feet_per_second_t{ state.velocity }, units::degree_t{ state.angle } };
    // frc::SwerveModuleState::Optimize(wpistate, frc::Rotation2d(units::degree_t{ m_Angle }));
    // auto optimized = CowLib::CowSwerveModuleState::FromWPI(wpistate);

    frc::SmartDashboard::PutNumber("Module " + std::to_string(m_Id) + " optimized velocity", optimized.velocity);
    frc::SmartDashboard::PutNumber("Module " + std::to_string(m_Id) + " optimized angle", optimized.angle);
    frc::SmartDashboard::PutNumber("Module " + std::to_string(m_Id) + " before opti angle", state.angle);
    frc::SmartDashboard::PutNumber("Module " + std::to_string(m_Id) + " current angle in deg", m_Angle);
    frc::SmartDashboard::PutNumber("Module " + std::to_string(m_Id) + " rotation motor position (turns)",
                                   m_RotationMotor->GetPosition());

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "Module %d velocity: %f target angle: %f current angle: %f\n",
    //                           m_Id,
    //                           optimized.velocity,
    //                           optimized.angle,
    //                           m_Angle);

    // auto optimized = state;
    // frc::SwerveModuleState optimized = state;

    double percentOutput = optimized.velocity / CONSTANT("SWERVE_MAX_SPEED");

    m_DriveControlRequest.PercentOut = percentOutput;

    // auto velTPS = CowLib::Conversions::FPSToFalcon(optimized.velocity,
    //                                                CONSTANT("WHEEL_CIRCUMFERENCE"),
    //                                                CONSTANT("SWERVE_DRIVE_GEAR_RATIO"));

    // m_DriveMotor->GetInternalTalon()->GetSimState().SetRotorVelocity(units::turns_per_second_t{ velTPS });
    // m_DriveMotor->GetInternalTalon()->GetSimState().SetRawRotorPosition(
    //     units::turn_t{ m_DriveMotor->GetPosition() + velTPS / 50.0 });

    // = units::turns_per_second_t{ CowLib::Conversions::FPSToFalcon(optimized.velocity,
    //                                                               CONSTANT("WHEEL_CIRCUMFERENCE"),
    //                                                               CONSTANT("SWERVE_DRIVE_GEAR_RATIO")) };

    // Don't rotate for low speeds
    double targetAngle;

    if (fabs(optimized.velocity) <= CONSTANT("SWERVE_MAX_SPEED") * 0.01)
    {
        targetAngle = m_PreviousAngle;
    }
    else
    {
        targetAngle = optimized.angle;
    }

    m_PreviousAngle = targetAngle;

    // m_RotationControlRequest.Position = targetAngle * CONSTANT("SWERVE_ROTATION_GEAR_RATIO") / 360.0;
    m_Angle = targetAngle;

    // m_RotationMotor->GetInternalTalon()->GetSimState().SetRawRotorPosition(
    //     units::turn_t{ CowLib::Conversions::DegreesToFalcon(targetAngle, CONSTANT("SWERVE_ROTATION_GEAR_RATIO")) });
}

/**
 * @brief Resets PID constants
 */
void SwerveModule::ResetConstants()
{
    // Percent output so no use
    // m_DriveMotor->SetPID(CONSTANT("SWERVE_DRIVE_P"),
    //                      CONSTANT("SWERVE_DRIVE_I"),
    //                      CONSTANT("SWERVE_DRIVE_D"),
    //                      CONSTANT("SWERVE_DRIVE_F"));
    m_RotationMotor->SetPID(CONSTANT("SWERVE_ANGLE_P"), CONSTANT("SWERVE_ANGLE_I"), CONSTANT("SWERVE_ANGLE_D"));
}

/**
 * @brief Resets encoder to absolute offset
 */
void SwerveModule::ResetEncoders()
{
    double absolutePosition = CowLib::Conversions::DegreesToFalcon(m_Encoder->GetAbsolutePosition() - m_EncoderOffset,
                                                                   CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "mod %n abs pos to set %f", m_Id, absolutePosition);

    m_RotationMotor->SetSensorPosition(absolutePosition);

    int errCode;
    do
    {
        errCode = m_DriveMotor->SetSensorPosition(0);
        if (errCode != 0)
        {
            CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "err code %d", errCode);
        }
    } while (errCode != 0);
}

/**
 * @brief Doesn't set anything but reads the current real world state
 */
void SwerveModule::Handle()
{
    // m_DriveMotor->Set(m_DriveControlRequest);
    // m_RotationMotor->Set(m_RotationControlRequest);

    // printf("Module %d abs enc angle: %f\n", m_Id, m_Encoder->GetAbsolutePosition());

    // // Update current positions once per loop
    // m_Velocity = CowLib::Conversions::FalconToFPS(m_DriveMotor->GetVelocity(),
    //                                               CONSTANT("WHEEL_CIRCUMFERENCE"),
    //                                               CONSTANT("SWERVE_DRIVE_GEAR_RATIO"));

    // // I think this is right...
    // m_Position = ((m_DriveMotor->GetPosition() * (1.0 / 1.0) / CONSTANT("SWERVE_DRIVE_GEAR_RATIO"))
    //               * CONSTANT("WHEEL_CIRCUMFERENCE"));

    // m_Angle
    //     = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));

    // SIM VALUES
    // m_Angle    = CowLib::Conversions::FalconToDegrees(m_RotationControlRequest.Position,
    //                                                CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));
    m_Velocity = m_DriveControlRequest.PercentOut * CONSTANT("SWERVE_MAX_SPEED");
    m_Position += m_Velocity / 50.0;
}

/**
 * @brief Helper function for optimize
 * @param scopeReference Current angle
 * @param newAngle Target angle
 * @return The closed angle within scope
 */
double SwerveModule::PlaceInAppropriate0To360Scope(double scopeReference, double newAngle)
{
    double lowerBound;
    double upperBound;
    double lowerOffset = std::fmod(scopeReference, 360);

    if (lowerOffset >= 0)
    {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
    }
    else
    {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }

    while (newAngle < lowerBound)
    {
        newAngle += 360;
    }

    while (newAngle > upperBound)
    {
        newAngle -= 360;
    }

    if (newAngle - scopeReference > 180)
    {
        newAngle -= 360;
    }
    else if (newAngle - scopeReference < -180)
    {
        newAngle += 360;
    }

    return newAngle;
}

/**
 * @brief Modified WPILib optimize function.
 * Minimize the change in heading the desired swerve module state would require by potentially
 * reversing the direction the wheel spins. Customized from WPILib's version to include
 * placing in appropriate scope for CTRE onboard control.
 * Port ofhttps://github.com/frc1678/C2022/blob/main/src/main/java/com/lib/util/CTREModuleState.java
 * @param desiredState The desired module state
 * @param currentAngle The current module angle
 * @return frc::SwerveModuleState
 */
CowLib::CowSwerveModuleState SwerveModule::Optimize(CowLib::CowSwerveModuleState desiredState, double currentAngle)
{
    double targetAngle = PlaceInAppropriate0To360Scope(currentAngle, desiredState.angle);
    double targetSpeed = desiredState.velocity;

    double delta = targetAngle - currentAngle;

    if (fabs(delta) > 90)
    {
        targetSpeed = -targetSpeed;

        // This is what it is in the original java
        // targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);

        targetAngle += ((delta > 90) ? -180 : 180);
    }

    return CowLib::CowSwerveModuleState{ targetSpeed, targetAngle };
}