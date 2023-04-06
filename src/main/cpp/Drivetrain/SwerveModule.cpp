#include "SwerveModule.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/kinematics/SwerveModuleState.h>

SwerveModule::SwerveModule(const int id,
                           const int driveMotor,
                           const int rotationMotor,
                           const int encoderId,
                           const double encoderOffset)
    : SwerveModuleInterface(id, encoderOffset)
{
    m_DriveMotor    = std::make_unique<CowLib::CowMotorController>(driveMotor, "cowdrive");
    m_RotationMotor = std::make_unique<CowLib::CowMotorController>(rotationMotor, "cowdrive");
    m_Encoder       = std::make_unique<CowLib::CowCANCoder>(encoderId);

    m_DriveControlRequest = { 0 };
    m_DriveMotor->OverrideBrakeMode(true);

    m_RotationMotor->SetInverted(true);
    m_RotationControlRequest = { 0 };

    m_PreviousAngle = 0;

    m_BrakeMode = true;

    ResetConstants();
    ResetEncoders();

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                              "Module %d abs encoder angle: %f  motor angle %f\n",
                              id,
                              m_Encoder->GetAbsolutePosition(),
                              m_RotationMotor->GetPosition());
    // frc::SmartDashboard::PutNumber("swerve/module " + std::to_string(m_Id) + "/absolute encoder angle",
    // m_Encoder->GetAbsolutePosition());
}

void SwerveModule::SetTargetState(CowLib::CowSwerveModuleState state, bool force)
{
    CowLib::CowSwerveModuleState optimized = Optimize(state, m_Angle);

    double percentOutput = optimized.velocity / CONSTANT("SWERVE_MAX_SPEED");

    m_DriveControlRequest.PercentOut = percentOutput;

    // Don't rotate for low speeds - unless we are e-braking
    double targetAngle;

    if (!force && fabs(optimized.velocity) <= CONSTANT("SWERVE_MAX_SPEED") * 0.01)
    {
        targetAngle = m_PreviousAngle;
    }
    else
    {
        targetAngle = optimized.angle;
    }

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "omtimized vel %f\n", optimized.velocity);
    // printf("optimized vel: %f\n", optimized.velocity);

    m_PreviousAngle = targetAngle;

    m_RotationControlRequest.Position = targetAngle * CONSTANT("SWERVE_ROTATION_GEAR_RATIO") / 360.0;
    // m_RotationControlRequest.FeedForward
    // = state.omega * 12 * 0.3 / (360 * (6380 / CONSTANT("SWERVE_ROTATION_GEAR_RATIO")) / 60);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/feedforward",
    //                                m_RotationControlRequest.FeedForward);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/omega", state.omega);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "module ");

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target velocity", optimized.velocity);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target angle", optimized.angle);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angle error", m_Angle - optimized.angle);
}

void SwerveModule::ResetConstants()
{
    m_RotationMotor->SetPID(CONSTANT("SWERVE_ANGLE_P"), CONSTANT("SWERVE_ANGLE_I"), CONSTANT("SWERVE_ANGLE_D"));

    // Percent output so not used
    // m_DriveMotor->SetPID(CONSTANT("SWERVE_DRIVE_P"),
    //                      CONSTANT("SWERVE_DRIVE_I"),
    //                      CONSTANT("SWERVE_DRIVE_D"),
    //                      CONSTANT("SWERVE_DRIVE_F"));
}

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

void SwerveModule::Handle()
{
    m_DriveMotor->Set(m_DriveControlRequest);
    m_RotationMotor->Set(m_RotationControlRequest);

    // printf("Module %d abs enc angle: %f\n", m_Id, m_Encoder->GetAbsolutePosition());

    // Update current positions once per loop
    m_Velocity = CowLib::Conversions::FalconToFPS(m_DriveMotor->GetVelocity(),
                                                  CONSTANT("WHEEL_CIRCUMFERENCE"),
                                                  CONSTANT("SWERVE_DRIVE_GEAR_RATIO"));

    // I think this is right...
    m_Position = ((m_DriveMotor->GetPosition() * (1.0 / 1.0) / CONSTANT("SWERVE_DRIVE_GEAR_RATIO"))
                  * CONSTANT("WHEEL_CIRCUMFERENCE"));

    m_Angle
        = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));

    m_AngularVelocity = m_RotationMotor->GetVelocity() * 360.0 / CONSTANT("SWERVE_ROTATION_GEAR_RATIO");

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/position", m_Position);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angle", m_Angle);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/velocity", m_Velocity);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angular velocity", m_AngularVelocity);
}

void SwerveModule::SetBrakeMode(bool brakeMode)
{
    if (brakeMode == m_BrakeMode)
    {
        return;
    }

    m_BrakeMode = brakeMode;

    if (m_BrakeMode)
    {
        m_DriveMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    }
    else
    {
        m_DriveMotor->SetNeutralMode(CowLib::CowMotorController::COAST);
    }
}
