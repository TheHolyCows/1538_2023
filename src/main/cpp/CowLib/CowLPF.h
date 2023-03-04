/*
 * CowLPF.h
 *
 *  Created on: Mar 31, 2016
 *      Author: kchau
 */

#ifndef SRC_COWLIB_COWLPF_H_
#define SRC_COWLIB_COWLPF_H_

namespace CowLib
{

    class CowLPF
    {
    private:
        double m_LPFBeta;
        double m_RawData;
        double m_SmoothData;
        CowLPF();

    public:
        CowLPF(double beta);
        double Calculate(double value);
        void UpdateBeta(double beta);
        void ReInit(double raw, double smooth);
        virtual ~CowLPF();
    };

} /* namespace CowLib */

#endif /* SRC_COWLIB_COWLPF_H_ */
