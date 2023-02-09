#include "CowSwerveKinematics.h"

#include "CowChassisSpeeds.h"
#include "Eigen/src/QR/HouseholderQR.h"
#include "frc/EigenCore.h"
#include "frc/geometry/Rotation2d.h"

#include <array>
#include <cstddef>
#include <cstdio>

namespace CowLib
{

    /**
    * @brief Creates a new CowSwerveKinematics object
    * @param wheelBase distance between the center of each swerve module on one edge
    */
    CowSwerveKinematics::CowSwerveKinematics(double wheelBase)
    {
        double p = wheelBase / 2.0;
        double n = wheelBase / -2.0;
        printf("Wheelbase: %f, %f, %f\n", wheelBase, p, n);

        m_ModulePositions[0] = Translation2d(p, p);
        m_ModulePositions[1] = Translation2d(p, n);
        m_ModulePositions[2] = Translation2d(n, p);
        m_ModulePositions[3] = Translation2d(n, n);

        for (int i = 0; i < NUM_MODULES; i++)
        {
            printf("Module %d: %f, %f\n", i, m_ModulePositions[i].X(), m_ModulePositions[i].Y());
        }

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
            // m_ModuleRotations[i] = frc::Rotation2d(m_ModulePositions[i].X(), m_ModulePositions[i].Y(), true);
            m_ModuleRotations[i] = Rotation2d(m_ModulePositions[i].X(), m_ModulePositions[i].Y());
            printf("Module %d: %f, %f\n", i, m_ModuleRotations[i].GetDegrees(), m_ModulePositions[i].X());
        }
        // print mod rotations
        for (int i = 0; i < NUM_MODULES; i++)
        {
            printf("Module %d: %f\n", i, m_ModuleRotations[i].GetDegrees());
        }
        m_PreviousCenterOfRotation = Translation2d(0, 0);
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
        auto &states = *moduleStates;

        auto highestModuleSpeed
            = std::max_element(states.begin(),
                               states.end(),
                               [](const auto &a, const auto &b) { return fabs(a.velocity) < fabs(b.velocity); })
                  ->velocity;

        if (highestModuleSpeed > maxSpeed)
        {
            for (auto &module : states)
            {
                module.velocity = module.velocity / highestModuleSpeed * maxSpeed;
            }
        }
    }

    /**
    * @brief Uses WPILib kinematics to calculate module states from chassisSpeeds
    * @param chassisSpeeds CowChassisSpeeds struct
    * @return std::array of 4 CowSwerveModuleStates
    */
    std::array<CowSwerveModuleState, 4>
    CowSwerveKinematics::CalculateModuleStates(const CowChassisSpeeds &chassisSpeeds,
                                               const Translation2d centerOfRotation)
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
                m_InverseKinematics(i * 2, 0)     = 1;
                m_InverseKinematics(i * 2, 1)     = 0;
                m_InverseKinematics(i * 2, 2)     = -m_ModulePositions[i].Y() + centerOfRotation.Y();
                m_InverseKinematics(i * 2 + 1, 0) = 0;
                m_InverseKinematics(i * 2 + 1, 1) = 1;
                m_InverseKinematics(i * 2 + 1, 2) = m_ModulePositions[i].X() - centerOfRotation.X();
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

    CowChassisSpeeds
    CowSwerveKinematics::CalculateChassisSpeeds(const std::array<CowSwerveModuleState, 4> &moduleStates)
    {
        frc::Matrixd<NUM_MODULES * 2, 1> moduleStateMatrix;

        for (int i = 0; i < NUM_MODULES; i++)
        {
            const CowSwerveModuleState &module = moduleStates[i];
            moduleStateMatrix(i * 2, 0)        = module.velocity * module.angle;
            moduleStateMatrix(i * 2 + 1, 0)    = module.velocity * module.angle;
        }

        Eigen::Vector3d chassisSpeedsVector = m_ForwardKinematics.solve(moduleStateMatrix);

        return { chassisSpeedsVector(0), chassisSpeedsVector(1), chassisSpeedsVector(2) };
    }

    CowChassisSpeeds CowSwerveKinematics::CalculateChassisSpeedsWithWheelConstraints(
        const std::array<CowSwerveModuleState, 4> &moduleStates)
    {
        // for (int i = 0; i < NUM_MODULES; i++)
        // {
        //     printf("Module %d: %f\n", i, m_ModuleRotations[i]->Degrees().value());
        // }
        frc::Matrixd<NUM_MODULES * 2, 3> constraintsMatrix;
        for (int i = 0; i < NUM_MODULES; i++)
        {
            printf("Loop %d mod state vel %f pos %f angle %f\n",
                   i,
                   moduleStates[i].velocity,
                   moduleStates[i].position,
                   moduleStates[i].angle);
            auto module = moduleStates[i];
            printf("got module\n");
            Rotation2d angle = Rotation2d(module.angle, true);
            printf("got angle\n");
            // auto a           = Rotation2d::FromDegrees(90);
            // auto inv = m_ModuleRotations[i].Inverse();
            auto angledeg = angle.GetDegrees();
            printf("angledeg %f\n", angledeg);
            printf("mod rots len = %d\n", (int) m_ModuleRotations.size());
            printf("sizeof m_ModuleRotations = %d:%d\n", (int) sizeof(m_ModuleRotations), sizeof(frc::Rotation2d));

            // frc::R ? otation2d cpy = m_ModuleRotations[i];
            //     // try
            //     // {
            //     frc::Rotation2d cpy = m_ModuleRotations.at(i);
            //     printf("cpy %f\n", cpy.Degrees().value());
            Rotation2d modrot = Rotation2d(m_ModuleRotations[i]);
            printf("got modrot\n");
            auto moddeg = modrot.GetDegrees();
            printf("moddeg %f\n", moddeg);
            // auto beta = Rotation2d(angle.GetDegrees() - m_ModuleRotations[i].Degrees().value() + 90, false);
            auto beta = angle.RotateBy(m_ModuleRotations[i].Inverse()).RotateBy(Rotation2d::FromDegrees(90));
            printf("after beta calc\n");

            constraintsMatrix(i * 2, 0) = angle.Cos();
            constraintsMatrix(i * 2, 1) = angle.Sin();
            constraintsMatrix(i * 2, 2) = -m_ModulePositions[i].Norm() * beta.Sin();

            constraintsMatrix(i * 2 + 1, 0) = -angle.Sin();
            constraintsMatrix(i * 2 + 1, 1) = angle.Cos();
            constraintsMatrix(i * 2 + 1, 2) = m_ModulePositions[i].Norm() * beta.Sin();
        }
        //     // catch (std::exception &e)
        //     // {
        //     //     printf("m_ModuleRotations[i] is null\n");
        //     // }
        // }

        printf("end for\n");
        printf("constraintsMatrix0: %f, %f, %f\n",
               constraintsMatrix(0, 0),
               constraintsMatrix(0, 1),
               constraintsMatrix(0, 2));
        printf("constraintsMatrix1: %f, %f, %f\n",
               constraintsMatrix(1, 0),
               constraintsMatrix(1, 1),
               constraintsMatrix(1, 2));
        // return CowChassisSpeeds{ 0, 0, 0 };

        // frc::Matrixd<NUM_MODULES * 2, 3> psuedoInv
        //     = constraintsMatrix.completeOrthogonalDecomposition().pseudoInverse();
        // auto psuedoInv = qr.solve(frc::Matrixd<3, 3>::Identity());
        //  var psuedoInv = constraintsMatrix.pseudoInverse();

        frc::Matrixd<NUM_MODULES * 2, 1> enforcedConstraints;
        for (int i = 0; i < NUM_MODULES; i++)
        {
            enforcedConstraints(i * 2, 0)     = moduleStates[i].velocity;
            enforcedConstraints(i * 2 + 1, 0) = 0;
        }

        // auto chassisSpeedsVector = psuedoInv * enforcedConstraints;
        auto chassisSpeedsVector = constraintsMatrix.householderQr().solve(enforcedConstraints);

        printf("vx %f vy %f omega %f\n", chassisSpeedsVector(0), chassisSpeedsVector(1), chassisSpeedsVector(2));

        auto returnSpeeds = CowChassisSpeeds{ chassisSpeedsVector(0), chassisSpeedsVector(1), chassisSpeedsVector(2) };
        printf("created return speeds\n");
        return returnSpeeds;
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