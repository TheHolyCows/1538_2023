#include "./CowSwerveOdometry.h"

#include "CowSwerveModuleState.h"
#include "frc/Timer.h"
#include "units/time.h"
#include "wpi/timestamp.h"

#include <cmath>

namespace CowLib
{

    CowSwerveOdometry::CowSwerveOdometry(CowSwerveKinematics *kinematics,
                                         Pose2d initialPose,
                                         std::array<double, 4> previousDistances)
    {
        m_Kinematics        = kinematics;
        m_Pose              = initialPose;
        m_ChassisSpeeds     = CowChassisSpeeds{ 0, 0, 0 };
        m_PreviousAngle     = initialPose.GetRotation();
        m_PreviousDistances = previousDistances;
    }

    CowSwerveOdometry::CowSwerveOdometry(CowSwerveKinematics *kinematics, Pose2d initialPose)
    {
        CowSwerveOdometry(kinematics, initialPose, { 0, 0, 0, 0 });
    }

    CowSwerveOdometry::CowSwerveOdometry(CowSwerveKinematics *kinematics)
    {
        CowSwerveOdometry(kinematics, Pose2d());
    }

    void CowSwerveOdometry::Reset(Pose2d pose, std::array<double, 4> previousDistances)
    {
        m_PreviousDistances = previousDistances;
        Reset(pose);
    }

    void CowSwerveOdometry::Reset(Pose2d pose)
    {
        m_Pose          = pose;
        m_ChassisSpeeds = CowChassisSpeeds{ 0, 0, 0 };
        m_PreviousAngle = pose.GetRotation();
    }

    Pose2d CowSwerveOdometry::GetPose() const
    {
        return m_Pose;
    }

    CowChassisSpeeds CowSwerveOdometry::GetChassisSpeeds() const
    {
        return m_ChassisSpeeds;
    }

    Pose2d CowSwerveOdometry::UpdateWithWheelConstraints(double currentTime,
                                                         Rotation2d gyroAngle,
                                                         std::array<CowSwerveModuleState, 4> moduleStates)
    {
        double period  = m_PreviousTime >= 0 ? currentTime - m_PreviousTime : 0.0;
        m_PreviousTime = currentTime;

        auto angle        = gyroAngle;
        auto chassisState = m_Kinematics->CalculuateChassisSpeedsWithWheelConstraints(moduleStates);
        return Pose2d();

        auto idealStates = m_Kinematics->CalculateModuleStates(chassisState);

        double average = 0.0;
        for (int i = 0; i < (int) moduleStates.size(); i++)
        {
            double ratio = Rotation2d::FromDegrees(moduleStates[i].angle - idealStates[i].angle).Cos()
                           * (moduleStates[i].position - m_PreviousDistances[i]) / (idealStates[i].velocity * period);

            if (std::isnan(ratio) || std::isinf(ratio) || fabs(idealStates[i].velocity) < 0.01)
            {
                ratio = 1.0;
            }

            average                = average + ratio;
            m_PreviousDistances[i] = moduleStates[i].position;
        }
        average /= 4.0;

        auto newPose = Pose2d::Exp(Twist2d(chassisState.vx * period * average,
                                           chassisState.vy * period * average,
                                           chassisState.omega * period * average));

        m_ChassisSpeeds = chassisState;

        printf("prev pose: %s\nnew componenet %s \n", m_Pose.ToString().c_str(), newPose.ToString().c_str());

        m_Pose          = Pose2d(m_Pose.GetX() + newPose.GetX(), m_Pose.GetY() + newPose.GetY(), angle);
        m_PreviousAngle = angle;

        return m_Pose;
    }

    Pose2d CowSwerveOdometry::Update(double gyroAngle, std::array<CowSwerveModuleState, 4> moduleStates)
    {
        return UpdateWithWheelConstraints(frc::Timer::GetFPGATimestamp().value(),
                                          Rotation2d::FromDegrees(gyroAngle),
                                          moduleStates);
    }

} // namespace CowLib
