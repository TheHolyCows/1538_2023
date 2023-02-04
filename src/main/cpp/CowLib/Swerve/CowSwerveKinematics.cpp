#include "CowSwerveKinematics.h"

#include "frc/EigenCore.h"

#include <array>

namespace CowLib
{

    /**
    * @brief Creates a new CowSwerveKinematics object
    * @param wheelBase distance between the center of each swerve module on one edge
    */
    CowSwerveKinematics::CowSwerveKinematics(double wheelBase)
    {
        auto p = wheelBase / 2.0;
        auto n = wheelBase / -2.0;

        m_ModulePositions
            = { Translation2d{ p, p }, Translation2d{ p, n }, Translation2d{ n, p }, Translation2d{ n, n } };

        for (int i = 0; i < NUM_MODULES; i++)
        {
            // clang-format off
            m_InverseKinematics.template block<2, 3>(i * 2, 0) <<
                1, 0, -m_ModulePositions[i].Y(),
                0, 1, +m_ModulePositions[i].X();
            // clang-format on
        }

        m_ForwardKinematics = m_InverseKinematics.householderQr();

        m_ModuleStates
            = { CowSwerveModuleState(), CowSwerveModuleState(), CowSwerveModuleState(), CowSwerveModuleState() };
        for (int i = 0; i < NUM_MODULES; i++)
        {
            m_ModuleRotations[i] = Rotation2d(m_ModulePositions[i].X(), m_ModulePositions[i].Y(), true);
        }
        m_PreviousCenterOfRotation = Translation2d();
    }

    CowSwerveKinematics::~CowSwerveKinematics()
    {
    }

    /**
    * @brief Re-normalizes wheel speeds if any individual speed is above maxSpeed
    * @param moduleStates Reference to std::array of CowSwerveModuleStates (mutated)
    * @param maxSpeed Maximum attainable speed (feet per second)
    */
    void CowSwerveKinematics::DesaturateSpeeds(std::array<CowSwerveModuleState, 4> *moduleStates, double maxSpeed)
    {
        // auto &states = *moduleStates;

        // auto highestModuleSpeed
        //     = std::max_element(states.begin(),
        //                        states.end(),
        //                        [](const auto &a, const auto &b) { return fabs(a.velocity) < fabs(b.velocity); })
        //           ->velocity;

        // if (highestModuleSpeed > maxSpeed)
        // {
        //     for (auto &module : states)
        //     {
        //         module.velocity = module.velocity / highestModuleSpeed * maxSpeed;
        //     }
        // }
    }

    /**
    * @brief Uses WPILib kinematics to calculate module states from chassisSpeeds
    * @param chassisSpeeds CowChassisSpeeds struct
    * @return std::array of 4 CowSwerveModuleStates
    */
    std::array<CowSwerveModuleState, 4>
    CowSwerveKinematics::CalculateModuleStates(const CowChassisSpeeds &chassisSpeeds,
                                               const Translation2d centerOfRotation) const
    {
        if (chassisSpeeds.vx == 0 && chassisSpeeds.vy == 0 && chassisSpeeds.omega == 0)
        {
            for (int i = 0; i < NUM_MODULES; i++)
            {
                m_ModuleStates[i].velocity = 0;
            }

            return m_ModuleStates;
        }

        // We have a new center of rotation. We need to compute the matrix again.
        if (m_PreviousCenterOfRotation != centerOfRotation)
        {
            for (int i = 0; i < NUM_MODULES; i++)
            {
                // clang-format off
                m_InverseKinematics.template block<2, 3>(i * 2, 0) =
                    frc::Matrixd<2, 3>{
                    {1, 0, (-m_ModulePositions[i].Y() + centerOfRotation.Y())},
                    {0, 1, (+m_ModulePositions[i].X() - centerOfRotation.X())}};
                // clang-format on
            }
            m_PreviousCenterOfRotation = centerOfRotation;
        }

        Eigen::Vector3d chassisSpeedsVector{ chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega };

        frc::Matrixd<NUM_MODULES * 2, 1> moduleStateMatrix = m_InverseKinematics * chassisSpeedsVector;

        for (int i = 0; i < NUM_MODULES; i++)
        {
            double vx = moduleStateMatrix(i * 2, 0);
            double vy = moduleStateMatrix(i * 2 + 1, 0);

            auto speed    = std::hypot(vx, vy);
            auto rotation = Rotation2d(vx, vy, true);

            m_ModuleStates[i] = { speed, rotation.GetDegrees() };
        }

        return m_ModuleStates;
    }

    CowChassisSpeeds CowSwerveKinematics::CalculateChassisSpeeds(std::array<CowSwerveModuleState, 4> &moduleStates)
    {
        frc::Matrixd<NUM_MODULES * 2, 1> moduleStateMatrix;

        for (int i = 0; i < NUM_MODULES; i++)
        {
            CowSwerveModuleState module     = moduleStates[i];
            moduleStateMatrix(i * 2, 0)     = module.velocity * module.angle;
            moduleStateMatrix(i * 2 + 1, 0) = module.velocity * module.angle;
        }

        Eigen::Vector3d chassisSpeedsVector = m_ForwardKinematics.solve(moduleStateMatrix);

        return { chassisSpeedsVector(0), chassisSpeedsVector(1), chassisSpeedsVector(2) };
    }

    CowChassisSpeeds
    CowSwerveKinematics::CalculuateChassisSpeedsWithWheelConstraints(std::array<CowSwerveModuleState, 4> &moduleStates)
    {
        frc::Matrixd<NUM_MODULES * 2, 3> constraintsMatrix;
        for (int i = 0; i < NUM_MODULES; i++)
        {
            auto module      = moduleStates[i];
            Rotation2d angle = Rotation2d(module.angle, true);

            auto beta = angle.RotateBy(m_ModuleRotations[i].Inverse()).RotateBy(Rotation2d::FromDegrees(90));

            constraintsMatrix(i * 2, 0) = angle.Cos();
            constraintsMatrix(i * 2, 1) = angle.Sin();
            constraintsMatrix(i * 2, 2) = -m_ModulePositions[i].Norm() * beta.Sin();

            constraintsMatrix(i * 2 + 1, 0) = -angle.Sin();
            constraintsMatrix(i * 2 + 1, 1) = angle.Cos();
            constraintsMatrix(i * 2 + 1, 2) = m_ModulePositions[i].Norm() * beta.Sin();
        }

        auto psuedoInv = constraintsMatrix.completeOrthogonalDecomposition().pseudoInverse();
        //  var psuedoInv = constraintsMatrix.pseudoInverse();

        frc::Matrixd<NUM_MODULES * 2, 1> enforcedConstraints;
        for (int i = 0; i < NUM_MODULES; i++)
        {
            enforcedConstraints(i * 2, 0)     = moduleStates[i].velocity;
            enforcedConstraints(i * 2 + 1, 0) = 0;
        }

        auto chassisSpeedsVector = psuedoInv * enforcedConstraints;

        return { chassisSpeedsVector(0), chassisSpeedsVector(1), chassisSpeedsVector(2) };
    }

    std::array<Translation2d, 4> CowSwerveKinematics::GetModulePositions()
    {
        return m_ModulePositions;
    }

    /**
    * @brief Retrieves the internal kinematics instance
    * @return WPILib SwerveDriveKinematics
    */
    frc::SwerveDriveKinematics<4> *CowSwerveKinematics::GetInternalKinematics()
    {
        // return m_Kinematics;
    }

} // namespace CowLib