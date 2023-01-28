#include "CowExponentialFilter.h"

namespace CowLib
{

    CowExponentialFilter::CowExponentialFilter(double exponent)
    {
        m_Exponent = exponent;
    }

    double CowExponentialFilter::Filter(double input)
    {
        input = pow(abs(input), m_Exponent) * input / abs(input);
    }

    void CowExponentialFilter::Reset(double newExponent)
    {
        m_Exponent = newExponent;
    }

} /* namespace CowLib */
