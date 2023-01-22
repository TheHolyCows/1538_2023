#ifndef __SWERVE_MODULE_H__
#define __SWERVE_MODULE_H__

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowCANCoder.h"
#include "../CowLib/CowLogger.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/Swerve/CowSwerveKinematics.h"
#include "../CowLib/Swerve/CowSwerveModulePosition.h"
#include "../CowLib/Swerve/CowSwerveModuleState.h"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DataLogManager.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <units/angle.h>
#include <units/velocity.h>

class SwerveModule
{
private:
    int m_Id;

    double m_EncoderOffset;

    CowLib::CowMotorController *m_DriveMotor;
    CowLib::CowMotorController *m_RotationMotor;

    ctre::phoenixpro::controls::DutyCycleOut m_DriveControlRequest{ 0 };
    ctre::phoenixpro::controls::PositionDutyCycle m_RotationControlRequest{ 0_tr };

    CowLib::CowCANCoder *m_Encoder;

    double m_Velocity;
    double m_Position;
    double m_Angle;

    double m_PreviousAngle;

    void ResetToAbsolute();

    // Direct port of https://github.com/frc1678/C2022/blob/main/src/main/java/com/lib/util/CTREModuleState.java
    static double PlaceInAppropriate0To360Scope(double scopeReference, double newAngle);
    static CowLib::CowSwerveModuleState Optimize(CowLib::CowSwerveModuleState desiredState, double currentAngle);

public:
    SwerveModule(int id, int driveMotor, int rotationMotor, int encoderId, double encoder_offset);
    ~SwerveModule();

    int GetId() { return m_Id; }

    CowLib::CowSwerveModuleState GetState();
    CowLib::CowSwerveModulePosition GetPosition();

    void SetTargetState(CowLib::CowSwerveModuleState state);

    void ResetConstants();

    void ResetEncoders();

    void Handle();
};

#endif /* __SWERVE_MODULE_H */