#include "CowExponentialFilter.h"
#include "math.h"

namespace CowLib
{

    CowExponentialFilter::CowExponentialFilter(double Exponent)
        : m_Exponent(Exponent)
    {
    }

    double CowExponentialFilter::Filter(double Input)
    {
        Input = pow(abs(Input), m_Exponent) * Input/abs(Input);
    }

    void CowExponentialFilter::ResetConstants()
    {
        
    }

    CowExponentialFilter::~CowExponentialFilter()
    {
    }


} /* namespace CowLib */
