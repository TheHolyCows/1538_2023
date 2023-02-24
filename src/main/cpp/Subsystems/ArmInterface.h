//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_INTERFACE_H_
#define SRC_SUBSYSTEMS_ARM_INTERFACE_H_

class ArmInterface
{
public:
    /**
     * @brief Default destructor
     * 
     */
    virtual ~ArmInterface() = default;

    /**
     * @brief Will rotate the arm to the specified angle
     * 
     * @param angle The desired angle in degrees to rotate to
     */
    void SetAngle(const double angle);

    /**
     * @brief Will set the telescoping position of the arm
     * 
     * @param position The desired position of the arm in inches
     */
    void SetTelescopePosition(const double position);

    /**
     * @brief Will return the current angle of the arm
     * 
     * @return double The current angle of the arm in degrees 
     */
    inline double GetAngle() const { return m_CurrentConfig.angle; }

    /**
     * @brief Will get the telescoping
     * 
     * @return double The desired telescoping position of the arm in inches
     */
    inline double GetTelescopePosition() const { return m_CurrentConfig.ext; }

    /**
     * @brief Will reset the PID values for both rotation and telescope motors
     * with the values defined in the CowConstants
     * 
     */
    virtual void ResetConstants() = 0;

    /**
     * @brief Will set the motors to their specified values
     * 
     */
    virtual void Handle() = 0;

    /**
     * @brief Will set the minimum or maximum angle depending on the current position
     * Will be called periodically.
     * 
     */
    virtual void CheckMinMax() = 0;

    /**
     * @brief Will set the angle of the arm to its zero position, which is the midpoint between
     * the set minimum and maximum angles of the arm.
     * 
     * This method assumes that the min and max angles are set correctly.
     * 
     */
    // virtual void ZeroSensors() = 0;

    /**
     * @brief The struct representing an angle and position of the arm
     * 
     */
    struct ArmConfig
    {
        double angle;
        double ext;
    };

protected:
    /**
     * @brief Will set the angle rotation motor's angle
     * 
     * @param angle The angle to set to
     */
    virtual void SetArmAngle(double angle) = 0;

    /**
     * @brief Will set the telescope motor's position
     * 
     * @param pos The ext to set to
     */
    virtual void SetArmExtension(double ext) = 0;

    // The Current Angle and Pos of the Arm
    ArmConfig m_CurrentConfig;

    // The Minimum Angle of the Arm
    double m_MinAngle;
    // The Maximum Angle of the Arm
    double m_MaxAngle;
    // The Minimum Pos of the Arm
    double m_MinPos;
    // The Maximum Pos of the Arm
    double m_MaxPos;
    // max angle of Wrist
    double m_WristMaxAngle;

    // constants for calculations
    double m_FrameHeight;
    double m_ClawLen;

private:
    /**
     * @brief Will safely set m_CurrentConfig with the specified angle
     * 
     * @param angle The angle to set the arm to
     */
    void SetSafeAngle(const double angle);

    /**
     * @brief Will safely set m_CurrentConfig with the specified position
     * 
     * @param position 
     */
    void SetSafePos(const double position);
};

#endif /* SRC_SUBSYSTEMS_ARM_INTERFACE_H_ */