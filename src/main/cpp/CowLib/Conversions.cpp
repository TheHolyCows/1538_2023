// Based on the conversions from 1678's 2022 codebase

#include "Conversions.h"

namespace CowLib
{

    namespace Conversions
    {
        /**
         * @brief Converts falcon units to degrees
         *
         * @param counts Falcon units
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return Degrees of rotation of mechanism
         */
        double FalconToDegrees(double counts, double gearRatio)
        {
            return counts * (360.0 / (gearRatio * 1));
        }

        /**
         * @brief Converts degrees to falcon units
         *
         * @param degrees Degrees of rotation of mechanism
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return Falcon units
         */
        double DegreesToFalcon(double degrees, double gearRatio)
        {
            return degrees * (gearRatio) / 360.0;
        }

        /**
         * @brief Converts falcon velocity units to mechanism RPM
         *
         * @param velocityCounts Falcon velocity units
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return RPM of mechanism
         */
        double FalconToRPM(double velocityCounts, double gearRatio)
        {
            double motorRPM = velocityCounts * (600.0 / 1.0);

            return motorRPM / gearRatio;
        }

        /**
         * @brief Converts mechanism RPM to falcon velocity units
         *
         * @param rpm RPM of mechanism
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return Falcon velocity counts
         */
        double RPMToFalcon(double rpm, double gearRatio)
        {
            double motorRPM = rpm * gearRatio;

            return motorRPM * (1.0 / 600.0);
        }

        /**
         * @brief Converts falcon units to feet per second
         *
         * @param velocityCounts Falcon velocity units
         * @param circumference Wheel circumference in feet
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return Velocity of mechanism in feet per second
         */
        double FalconToFPS(double velocityCounts, double circumference, double gearRatio)
        {
            double wheelRPM = FalconToRPM(velocityCounts, gearRatio);

            return (wheelRPM * circumference) / 60;
        }

        /**
         * @brief Converts feet per second of mechanism to falcon velocity counts
         *
         * @param velocity Mechanism velocity in feet per second
         * @param circumference Wheel circumference in feet
         * @param gearRatio Gear ratio between falcon and mechanism
         * @return Falcon velocity units
         */
        double FPSToFalcon(double velocity, double circumference, double gearRatio)
        {
            double wheelRPM = ((velocity * 60) / circumference);

            return RPMToFalcon(wheelRPM, gearRatio);
        }

        /**
         * @brief converts falcon units to inches
         * 
         * @param counts Encoder units
         * @param gearRatio gearing ratio of motor
        */
        double FalconToInches(double counts, double gearRatio)
        {
            return counts * gearRatio;
        }

        /**
         * @brief converts inches to falcon encoder units
         * 
         * @param inches inches traveled/exteded
         * @param gearRatio gearing ratio of motor
        */
        double InchesToFalcon(double inches, double gearRatio)
        {
            return inches / gearRatio;
        }

    } // namespace Conversions

} // namespace CowLib