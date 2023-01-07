/*
 * CowPID.h
 *
 *  Created on: Jan 11, 2016
 *      Author: kchau
 */

#ifndef SRC_COWLIB_COWPID_H_
#define SRC_COWLIB_COWPID_H_

#include <cfloat>
#include <cmath>
#include <string>

namespace CowLib
{

    class CowPID
    {
    public:
        CowPID(double Kp, double Ki, double Kd, double Kf);
        virtual ~CowPID();
        double Calculate(double input);
        double GetError();
        double GetSetpoint();
        void SetSetpoint(double setpoint);

        void Reset();
        bool OnTarget(double tolerance);
        void ResetIntegrator();
        std::string GetState();

        void SetContinuous(bool continuous);
        void SetInputRange(double min, double max);
        void SetOutputRange(double min, double max);

        void UpdateConstants(double Kp, double Ki, double Kd, double Kf);
        void UpdatePConstant(double Kp);

    private:
        CowPID();

        double m_P;             // factor for "proportional" control
        double m_I;             // factor for "integral" control
        double m_D;             // factor for "derivative" control
        double m_F;             // factor for "feedforward"
        double m_maximumOutput; // |maximum output|
        double m_minimumOutput; // |minimum output|
        double m_maximumInput;  // maximum input - limit setpoint to this
        double m_minimumInput;  // minimum input - limit setpoint to this
        bool m_continuous;      // do the endpoints wrap around? eg. Absolute encoder
        double m_prevError;     // the prior sensor input (used to compute velocity)
        double m_totalError;    // the sum of the errors for use in the integral calc
        double m_setpoint;
        double m_error;
        double m_result;
        double m_last_input;
    };

} /* namespace CowLib */

#endif /* SRC_COWLIB_COWPID_H_ */
