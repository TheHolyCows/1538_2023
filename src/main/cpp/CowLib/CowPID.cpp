/*
 * CowPID.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: kchau
 */

#include "CowPID.h"

#include <iostream>
#include <sstream>

namespace CowLib
{

    CowPID::CowPID(double Kp, double Ki, double Kd, double Kf)
        : m_P(Kp),
          m_I(Ki),
          m_D(Kd),
          m_F(Kf),
          m_maximumOutput(1.0),
          m_minimumOutput(-1.0),
          m_maximumInput(0),
          m_minimumInput(0),
          m_continuous(false),
          m_prevError(0),
          m_totalError(0),
          m_setpoint(0),
          m_error(0),
          m_result(0),
          m_last_input(NAN)
    {
    }

    CowPID::~CowPID()
    {
    }

    double CowPID::Calculate(double input)
    {
        m_last_input = input;
        m_error      = m_setpoint - input;
        if (m_continuous)
        {
            if (fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
            {
                if (m_error > 0)
                {
                    m_error = m_error - m_maximumInput + m_minimumInput;
                }
                else
                {
                    m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }
        }

        if ((m_error * m_P < m_maximumOutput) && (m_error * m_P > m_minimumOutput))
        {
            m_totalError += m_error;
        }
        else
        {
            m_totalError = 0;
        }

        m_result    = (m_setpoint * m_F) + (m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError));
        m_prevError = m_error;

        if (m_result > m_maximumOutput)
        {
            m_result = m_maximumOutput;
        }
        else if (m_result < m_minimumOutput)
        {
            m_result = m_minimumOutput;
        }
        return m_result;
    }

    double CowPID::GetError()
    {
        return m_error;
    }

    double CowPID::GetSetpoint()
    {
        return m_setpoint;
    }

    void CowPID::SetSetpoint(double setpoint)
    {
        if (m_maximumInput > m_minimumInput)
        {
            if (setpoint > m_maximumInput)
            {
                m_setpoint = m_maximumInput;
            }
            else if (setpoint < m_minimumInput)
            {
                m_setpoint = m_minimumInput;
            }
            else
            {
                m_setpoint = setpoint;
            }
        }
        else
        {
            m_setpoint = setpoint;
        }
    }

    void CowPID::Reset()
    {
        m_last_input = NAN;
        m_prevError  = 0;
        m_totalError = 0;
        m_result     = 0;
        m_setpoint   = 0;
    }

    bool CowPID::OnTarget(double tolerance)
    {
        // double toleranceDelta = fabs(m_last_input - m_setpoint);
        // std::cout << "Target: " << toleranceDelta << ", last in: " << m_last_input << " tolerance: " << tolerance << std::endl;
        return m_last_input != NAN && fabs(m_last_input - m_setpoint) < tolerance;
    }

    std::string CowPID::GetState()
    {
        std::stringstream lStateStream;

        lStateStream << "Error: " << m_error << "\n";
        return lStateStream.str();
    }

    void CowPID::SetContinuous(bool continuous)
    {
        m_continuous = continuous;
    }

    void CowPID::SetInputRange(double min, double max)
    {
        m_minimumInput = min;
        m_maximumInput = max;
        SetSetpoint(m_setpoint);
    }

    void CowPID::SetOutputRange(double min, double max)
    {
        m_minimumOutput = min;
        m_maximumOutput = max;
    }

    void CowPID::ResetIntegrator()
    {
        m_totalError = 0;
    }

    void CowPID::UpdateConstants(double Kp, double Ki, double Kd, double Kf)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
        m_F = Kf;

        ResetIntegrator();
    }

    void CowPID::UpdatePConstant(double Kp)
    {
        m_P = Kp;
    }

} /* namespace CowLib */
