//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_MOCK_H_
#define SRC_SUBSYSTEMS_ARM_MOCK_H_

#include "../CowLib/CowLogger.h"
#include "ArmInterface.h"

#include <sstream>

class ArmMock : public ArmInterface
{
public:
    ArmMock() = default;

    /**
     * @brief Will reset the PID values for both rotation and telescope motors
     * with the values defined in the CowConstants
     * 
     */
    void ResetConstants() override { CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Resetting Arm Constants"); }

    /**
     * @brief Will set the motors to their specified values
     * 
     */
    void Handle() override
    {
        // Maybe don't log here since this is called so often
        // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Handling Arm Code");
    }

    /**
     * @brief Will set the minimum or maximum angle depending on the current position
     * Will be called periodically. 
     * 
     */
    void CheckMinMax() override { CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Checking MinMax"); }

    /**
     * @brief Will set the angle of the arm to its zero position, which is the midpoint between
     * the set minimum and maximum angles of the arm.
     * 
     * This method assumes that the min and max angles are set correctly.
     * 
     */
    // void ZeroSensors() override { CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Zeroing Sensors"); }

private:
    /**
     * @brief Will rotate the arm to the specified angle
     * 
     * @param angle The desired angle in degrees to rotate to
     */
    void SetArmAngle(const double angle) override
    {
        std::stringstream ss;
        ss << "Setting Arm Angle to: " << angle << " degrees";
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, ss.str().c_str());
    }

    /**
     * @brief Will set the telescoping position of the arm
     * 
     * @param position The desired position of the arm in inches
     */
    void SetArmPosition(const double pos) override
    {
        std::stringstream ss;
        ss << "Setting Arm Pos to: " << pos << " degrees";
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, ss.str().c_str());
    }
};

#endif //SRC_SUBSYSTEMS_ARM_MOCK_H_