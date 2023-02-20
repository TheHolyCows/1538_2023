#ifndef __SWERVE_MODULE_SIM_H__
#define __SWERVE_MODULE_SIM_H__

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowCANCoder.h"
#include "../CowLib/CowLogger.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/Swerve/CowSwerveKinematics.h"
#include "../CowLib/Swerve/CowSwerveModulePosition.h"
#include "../CowLib/Swerve/CowSwerveModuleState.h"
#include "SwerveModuleInterface.h"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DataLogManager.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <memory>
#include <units/angle.h>
#include <units/velocity.h>

class SwerveModuleSim : public SwerveModuleInterface
{
private:
    const double m_Accel;
    const double m_MaxAngularVelocity;
    const double m_AngularAccel;

    double m_TargetAngle;
    double m_TargetVelocity;

    double m_PreviousAngle;

    std::unique_ptr<CowLib::CowCANCoder> m_Encoder;

public:
    SwerveModuleSim(const int id,
                    const double accel,
                    const double maxAngularVelocity,
                    const double angularAccel,
                    const double encoderOffset);

    void SetTargetState(CowLib::CowSwerveModuleState state, bool force = false) override;

    void ResetConstants() override;

    void ResetEncoders() override;

    void Handle() override;
};

#endif /* __SWERVE_MODULE_SIM_H */