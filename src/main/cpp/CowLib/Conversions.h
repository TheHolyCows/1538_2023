// Based on the conversions from 1678's 2022 codebase

#ifndef __CONVERSIONS_H__
#define __CONVERSIONS_H__

namespace CowLib
{

    namespace Conversions
    {
        double FalconToDegrees(double counts, double gearRatio);
        double DegreesToFalcon(double degrees, double gearRatio);

        double FalconToRPM(double velocityCounts, double gearRatio);
        double RPMToFalcon(double rpm, double gearRatio);

        double FalconToFPS(double velocityCounts, double circumference, double gearRatio);
        double FPSToFalcon(double velocity, double circumference, double gearRatio);

        double FalconToInches(double counts, double gearRatio);
        double InchesToFalcon(double counts, double gearRatio);
    } // namespace Conversions

} // namespace CowLib

#endif /* __CONVERSIONS_H__ */