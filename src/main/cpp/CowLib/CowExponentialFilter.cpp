#include "CowExponentialFilter.h"

#include "math.h"

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

    // CowExponentialFilter::~CowExponentialFilter()
    // {
    // }

} /* namespace CowLib */
