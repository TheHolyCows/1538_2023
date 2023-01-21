#include "CowHolonomicController.h"

namespace CowLib
{

    CowHolonomicController::CowHolonomicController(double driveP,
                                                   double driveI,
                                                   double driveD,
                                                   double rotationP,
                                                   double rotationI,
                                                   double rotationD,
                                                   double maxRotationalVelocity,
                                                   double maxRotationalAccel)
    {
        m_XPIDController     = new frc2::PIDController(driveP, driveI, driveD);
        m_YPIDController     = new frc2::PIDController(driveP, driveI, driveD);
        m_ThetaPIDController = new frc::ProfiledPIDController<units::radian>(
            rotationP,
            rotationI,
            rotationD,
            frc::TrapezoidProfile<units::radian>::Constraints(units::degrees_per_second_t{ maxRotationalVelocity },
                                                              units::degrees_per_second_t{ maxRotationalAccel } / 1_s));

        m_HolonomicController
            = new frc::HolonomicDriveController(*m_XPIDController, *m_YPIDController, *m_ThetaPIDController);
    }

    CowHolonomicController::~CowHolonomicController()
    {
        delete m_XPIDController;
        delete m_YPIDController;
        delete m_ThetaPIDController;
        delete m_HolonomicController;
    }

    CowLib::CowChassisSpeeds
    CowHolonomicController::Calculate(const frc::Pose2d &pose, frc::Trajectory::State desiredState, double desiredAngle)
    {
        auto chassisSpeeds
            = m_HolonomicController->Calculate(pose, desiredState, frc::Rotation2d(units::degree_t{ desiredAngle }));
        return CowLib::CowChassisSpeeds::FromWPI(chassisSpeeds);
    }

    CowLib::CowChassisSpeeds CowHolonomicController::Calculate(const frc::Pose2d &currentPose,
                                                               const frc::Pose2d &desiredPose,
                                                               double maxLinearSpeed,
                                                               double desiredAngle)
    {
        auto chassisSpeeds = m_HolonomicController->Calculate(currentPose,
                                                              desiredPose,
                                                              units::feet_per_second_t{ maxLinearSpeed },
                                                              frc::Rotation2d(units::degree_t{ desiredAngle }));
        return CowLib::CowChassisSpeeds::FromWPI(chassisSpeeds);
    }

    frc::HolonomicDriveController *CowHolonomicController::GetInternalController()
    {
        return m_HolonomicController;
    }

} // namespace CowLib