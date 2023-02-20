#include "SwerveModuleSim.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/kinematics/SwerveModuleState.h>

SwerveModuleSim::SwerveModuleSim(const int id,
                                 const double accel,
                                 const double maxAngularVelocity,
                                 const double angularAccel,
                                 const double encoderOffset)
    : SwerveModuleInterface(id, encoderOffset),
      m_Accel(accel),
      m_MaxAngularVelocity(maxAngularVelocity),
      m_AngularAccel(angularAccel),
      m_TargetAngle(0),
      m_TargetVelocity(0),
      m_PreviousAngle(0)
{
    ResetConstants();
    ResetEncoders();
}

void SwerveModuleSim::SetTargetState(CowLib::CowSwerveModuleState state, bool force)
{
    CowLib::CowSwerveModuleState optimized = Optimize(state, m_Angle);

    m_TargetVelocity = optimized.velocity;

    // Don't rotate for low speeds - unless forced
    if (!force && fabs(optimized.velocity) <= CONSTANT("SWERVE_MAX_SPEED") * 0.01)
    {
        m_TargetAngle = m_PreviousAngle;
    }
    else
    {
        m_TargetAngle = optimized.angle;
    }

    m_PreviousAngle = m_TargetAngle;
}

void SwerveModuleSim::ResetConstants()
{
}

void SwerveModuleSim::ResetEncoders()
{
}

void SwerveModuleSim::Handle()
{
    frc::SmartDashboard::PutNumber("SwerveModuleSim/Module" + std::to_string(m_Id) + "/velocity", m_Velocity);
    frc::SmartDashboard::PutNumber("SwerveModuleSim/Module" + std::to_string(m_Id) + "/angle", m_Angle);

    // Change velocity based on accel
    if (fabs(m_TargetVelocity - m_Velocity) <= m_Accel * 0.02)
    {
        m_Velocity = m_TargetVelocity;
    }
    else if (m_TargetVelocity > m_Velocity)
    {
        m_Velocity += m_Accel;
    }
    else if (m_TargetVelocity < m_Velocity)
    {
        m_Velocity -= m_Accel;
    }

    // Accumulate to velocity
    m_Position += m_Velocity * 0.02;

    m_AngularVelocity = m_MaxAngularVelocity;
    if (fabs(m_TargetAngle - m_Angle) <= m_MaxAngularVelocity * 0.02)
    {
        m_Angle           = m_TargetAngle;
        m_AngularVelocity = fabs(m_TargetAngle - m_Angle);
    }
    else if (m_TargetAngle > m_Angle)
    {
        m_Angle += m_MaxAngularVelocity * 0.02;
    }
    else if (m_TargetAngle < m_Angle)
    {
        m_Angle -= m_MaxAngularVelocity * 0.02;
    }
}
