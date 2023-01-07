/*
 * CowTrapezoidalM.cpp
 *
 *  Created on: Feb 18, 2016
 *      Author: kchau
 */

#include "CowTrapezoidalM.h"

namespace CowLib
{

    CowTrapezoidalM::CowTrapezoidalM(double accelSlope, double decelSlope)
        : m_AccelSlope(0),
          m_DecelSlope(0),
          m_StartingPosition(0),
          m_Setpoint(0),
          m_AccelerationPhase(0)
    {
    }

    void CowTrapezoidalM::SetStartingPosition(double startPosition)
    {
        m_StartingPosition = startPosition;
    }

    void CowTrapezoidalM::SetSetpoint(double setPoint)
    {
        m_Setpoint          = setPoint;
        m_AccelerationPhase = 1;
    }

    double CowTrapezoidalM::Calculate(double currentPosition)
    {
        double totalDistance = m_Setpoint - m_StartingPosition;
        double distanceToGo  = m_Setpoint - currentPosition;

        double holdPeriod  = 1 - (1 * m_DecelSlope) - (1 * m_AccelSlope);
        double decelPeriod = 1 - holdPeriod - (1 * m_AccelSlope);
        double accelPeriod = 1 - holdPeriod - decelPeriod;

        double finalAccelPosition = totalDistance * accelPeriod;

        if (m_AccelerationPhase == 1)
        {
            if (distanceToGo < (finalAccelPosition))
            {
                double speed = (currentPosition / finalAccelPosition);
                if (speed == 0)
                {
                    speed = 0.1;
                }
                return speed;
            }
        }

        return 0;
    }

    CowTrapezoidalM::~CowTrapezoidalM()
    {
    }

} /* namespace CowLib */
