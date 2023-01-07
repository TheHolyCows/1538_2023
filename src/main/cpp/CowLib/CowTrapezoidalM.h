/*
 * CowTrapezoidalM.h
 *
 *  Created on: Feb 18, 2016
 *      Author: kchau
 */

#ifndef SRC_COWLIB_COWTRAPEZOIDALM_H_
#define SRC_COWLIB_COWTRAPEZOIDALM_H_

#include "CowLib.h"

#include <cmath>

namespace CowLib
{

    class CowTrapezoidalM
    {
    private:
        double m_AccelSlope;
        double m_DecelSlope;
        double m_StartingPosition;
        double m_Setpoint;
        uint8_t m_AccelerationPhase;

    public:
        CowTrapezoidalM(double accelSlope, double decelSlope);
        void SetStartingPosition(double startPosition);
        void SetSetpoint(double setPoint);
        double Calculate(double currentPosition);
        virtual ~CowTrapezoidalM();

    private:
        CowTrapezoidalM();
    };

} /* namespace CowLib */

#endif /* SRC_COWLIB_COWTRAPEZOIDALM_H_ */
