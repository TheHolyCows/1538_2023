/*
 * CowLPF.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: kchau
 */

#include "CowLPF.h"

namespace CowLib
{

    CowLPF::CowLPF(double beta)
        : m_LPFBeta(beta),
          m_RawData(0),
          m_SmoothData(0)
    {
    }

    double CowLPF::Calculate(double value)
    {
        m_RawData    = value;
        m_SmoothData = m_SmoothData - (m_LPFBeta * (m_SmoothData - m_RawData));

        return m_SmoothData;
    }

    void CowLPF::UpdateBeta(double beta)
    {
        m_RawData    = 0;
        m_SmoothData = 0;
        m_LPFBeta    = beta;
    }

    void CowLPF::ReInit(double raw, double smooth)
    {
        m_RawData    = raw;
        m_SmoothData = smooth;
    }

    CowLPF::~CowLPF()
    {
    }

} /* namespace CowLib */
