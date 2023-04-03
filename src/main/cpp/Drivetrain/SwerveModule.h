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

class SwerveModule : public SwerveModuleInterface
{
private:
    double m_PreviousAngle;

    std::unique_ptr<CowLib::CowMotorController> m_DriveMotor;
    std::unique_ptr<CowLib::CowMotorController> m_RotationMotor;

    CowLib::CowMotorController::PercentOutput m_DriveControlRequest;
    CowLib::CowMotorController::PositionPercentOutput m_RotationControlRequest;

    std::unique_ptr<CowLib::CowCANCoder> m_Encoder;

    bool m_BrakeMode;

public:
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
    SwerveModule(const int id,
                 const int driveMotor,
                 const int rotationMotor,
                 const int encoderId,
                 const double encoderOffset);

    /**
     * @brief Sets the desired module state to the given state after optimizing
     * @param state Target state
     * @param force force angle during low speeds
     */
    void SetTargetState(CowLib::CowSwerveModuleState state, bool force = false) override;

    void SetBrakeMode(bool brakeMode) override;

    void ResetConstants() override;

    void ResetEncoders() override;

    void Handle() override;
};

#endif /* __SWERVE_MODULE_H */