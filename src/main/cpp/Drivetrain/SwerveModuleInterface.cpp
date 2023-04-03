#include "SwerveModuleInterface.h"

SwerveModuleInterface::SwerveModuleInterface(const int id, const double encoderOffset)
    : m_Id(id),
      m_EncoderOffset(encoderOffset),
      m_Velocity(0),
      m_Position(0),
      m_Angle(0),
      m_AngularVelocity(0)
{
}

double SwerveModuleInterface::PlaceInAppropriate0To360Scope(double scopeReference, double newAngle)
{
    double lowerBound;
    double upperBound;
    double lowerOffset = std::fmod(scopeReference, 360);

    if (lowerOffset >= 0)
    {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
    }
    else
    {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }

    while (newAngle < lowerBound)
    {
        newAngle += 360;
    }

    while (newAngle > upperBound)
    {
        newAngle -= 360;
    }

    if (newAngle - scopeReference > 180)
    {
        newAngle -= 360;
    }
    else if (newAngle - scopeReference < -180)
    {
        newAngle += 360;
    }

    return newAngle;
}

CowLib::CowSwerveModuleState SwerveModuleInterface::Optimize(CowLib::CowSwerveModuleState desiredState,
                                                             double currentAngle)
{
    double targetAngle = PlaceInAppropriate0To360Scope(currentAngle, desiredState.angle);
    double targetSpeed = desiredState.velocity;

    double delta = targetAngle - currentAngle;

    if (fabs(delta) > 90)
    {
        targetSpeed = -targetSpeed;

        // This is what it is in the original java
        // targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);

        targetAngle += ((delta > 90) ? -180 : 180);
    }

    return CowLib::CowSwerveModuleState{ targetSpeed, targetAngle };
}

void SwerveModuleInterface::SetBrakeMode(bool brakeMode)
{
}
