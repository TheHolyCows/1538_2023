#include "SwerveModule.h"

/**
 * @brief Construct a new SwerveModule object
 *
 * @param id Module ID
 * @param driveMotor Drive motor ID
 *
 *
 * @param rotationMotor Rotation motor ID
 * @param encoderId CANCoder ID
 * @param encoderOffset Absolute encoder
 *
 * offset
 * @param locationX Module X translation
 * @param locationY Module Y translation
 */
SwerveModule::SwerveModule(int id, int driveMotor, int rotationMotor, int encoderId, double encoderOffset)
{
    m_Id = id;

    m_EncoderOffset = encoderOffset;

    m_DriveMotor    = new CowLib::CowMotorController(driveMotor);
    m_RotationMotor = new CowLib::CowMotorController(rotationMotor);
    m_Encoder       = new CowLib::CowCANCoder(encoderId);

    m_DriveMotor->SetControlMode(CowLib::CowMotorController::PERCENTVBUS);
    m_RotationMotor->SetControlMode(CowLib::CowMotorController::POSITION);

    m_RotationMotor->SetInverted(true);

    // init state
    m_Velocity = 0;
    m_Angle    = 0;

    ResetConstants();
    ResetEncoders();

    char *dbgMsg;
    asprintf(&dbgMsg, "Module %d abs encoder angle: %f", id, m_Encoder->GetAbsolutePosition());
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, dbgMsg);
}

SwerveModule::~SwerveModule()
{
    delete m_DriveMotor;
    delete m_RotationMotor;
    delete m_Encoder;
}

/**
 * @brief Returns the current module state
 *
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
 *
 * @param state Target state
 */
void SwerveModule::SetTargetState(CowLib::CowSwerveModuleState state)
{
    CowLib::CowSwerveModuleState optimized = Optimize(state, m_Angle);
    // frc::SwerveModuleState optimized = state;

    double percentOutput = optimized.velocity / CONSTANT("SWERVE_MAX_SPEED");

    m_DriveMotor->Set(percentOutput);

    // Don't rotate for low speeds
    double targetAngle;

    if (fabs(optimized.velocity) <= CONSTANT("SWERVE_MAX_SPEED") * 0.01)
    {
        targetAngle = m_PreviousAngle;
    }
    else
    {
        targetAngle = state.angle;
    }

    m_PreviousAngle = targetAngle;

    m_RotationMotor->Set(CowLib::Conversions::DegreesToFalcon(targetAngle, CONSTANT("SWERVE_ROTATION_GEAR_RATIO")));
}

/**
 * @brief Resets PID constants
 *
 */
void SwerveModule::ResetConstants()
{
    m_DriveMotor->SetPIDGains(CONSTANT("SWERVE_DRIVE_P"),
                              CONSTANT("SWERVE_DRIVE_I"),
                              CONSTANT("SWERVE_DRIVE_D"),
                              CONSTANT("SWERVE_DRIVE_F"),
                              1.0);
    m_RotationMotor->SetPIDGains(CONSTANT("SWERVE_ANGLE_P"),
                                 CONSTANT("SWERVE_ANGLE_I"),
                                 CONSTANT("SWERVE_ANGLE_D"),
                                 0,
                                 1.0);
}

/**
 * @brief Resets encoder to absolute offset
 *
 */
void SwerveModule::ResetEncoders()
{
    double absolutePosition = CowLib::Conversions::DegreesToFalcon(m_Encoder->GetAbsolutePosition() - m_EncoderOffset,
                                                                   CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));

    m_RotationMotor->GetInternalMotor()->SetSelectedSensorPosition(absolutePosition);
}

/**
 * @brief Doesn't set anything but reads the current real world state
 */
void SwerveModule::Handle()
{
    // Update current positions once per loop
    m_Velocity = CowLib::Conversions::FalconToFPS(m_DriveMotor->GetInternalMotor()->GetSelectedSensorVelocity(),
                                                  CONSTANT("WHEEL_CIRCUMFERENCE"),
                                                  CONSTANT("SWERVE_DRIVE_GEAR_RATIO"));

    // I think this is right...
    m_Position = (m_DriveMotor->GetPosition() * (600.0 / 2048.0) / CONSTANT("SWERVE_DRIVE_GEAR_RATIO"))
                 * CONSTANT("WHEEL_CIRCUMFERENCE");

    m_Angle
        = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));
}

/**
 * @brief Helper function for optimize
 *
 * @param scopeReference Current angle
 * @param newAngle Target angle
 *

 * * @return The closed angle within scope
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
 * Minimize the change in heading the desired swerve module state
 * would
 * require by potentially
 * reversing the direction the wheel spins. Customized from WPILib's version to
 * include
 * placing
 * in appropriate scope for CTRE onboard control.
 * Port of
 *
 * https://github.com/frc1678/C2022/blob/main/src/main/java/com/lib/util/CTREModuleState.java
 *
 * @param desiredState

 * * The desired module state
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