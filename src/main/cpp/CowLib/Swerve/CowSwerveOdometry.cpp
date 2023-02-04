#include "./CowSwerveOdometry.h"

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
        return Pose2d();
        // double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        // // System.out.println("Time: " + currentTimeSeconds);
        // m_prevTimeSeconds = currentTimeSeconds;

        // var angle        = gyroAngle;
        // var chassisState = m_kinematics.toChasisSpeedWheelConstraints(moduleStates);

        // var idealStates = m_kinematics.toSwerveModuleStates(chassisState);

        // // Project along ideal angles.
        // double average = 0.0;
        // for (int i = 0; i < moduleStates.length; ++i)
        // {
        //     double ratio = moduleStates[i].angle.rotateBy(idealStates[i].angle.inverse()).cos()
        //                    * (moduleStates[i].distanceMeters - m_previousDistances[i])
        //                    / (idealStates[i].speedMetersPerSecond * period);
        //     if (Double.isNaN(ratio) || Double.isInfinite(ratio) || Math.abs(idealStates[i].speedMetersPerSecond) < 0.01)
        //     {
        //         ratio = 1.0;
        //     }
        //     average                = average + ratio;
        //     m_previousDistances[i] = moduleStates[i].distanceMeters;
        // }
        // average = average / 4.0;

        // // System.out.println(chassisState);
        // SmartDashboard.putNumber("average", average);

        // var newPose = Pose2d.exp(new Twist2d(chassisState.vxMetersPerSecond * period * average,
        //                                      chassisState.vyMetersPerSecond * period * average,
        //                                      chassisState.omegaRadiansPerSecond * period * average));
        // // System.out.println("Translation: " + newPose);
        // m_velocity = chassisState;
        // // m_velocity.omegaRadiansPerSecond =
        // // m_previousAngle.inverse().rotateBy(gyroAngle).getRadians() / period;
        // m_poseMeters    = new Pose2d(m_poseMeters.transformBy(newPose).getTranslation(), angle);
        // m_previousAngle = angle;
        // m_field2d.setRobotPose(m_poseMeters.getTranslation().x(),
        //                        m_poseMeters.getTranslation().y(),
        //                        new edu.wpi.first.math.geometry.Rotation2d(m_poseMeters.getRotation().getDegrees()));

        // return m_poseMeters;
    }

    void CowSwerveOdometry::Update(double gyroAngle, std::array<CowSwerveModulePosition, 4> modulePositions)
    {
        return;
    }

} // namespace CowLib
