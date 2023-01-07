/*
 * Arm.h
 *
 *  Created on: Feb 18, 2018
 *      Author: kchau
 */

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"

#include <string>

class Arm
{
public:
    Arm(int motorControllerA,
        int motorControllerB,
        double maxSpeed,
        double upLimit,
        double downLimit,
        std::string name,
        bool changeDirectionA,
        bool changeDirectionB,
        double degreesPerTick,
        double peakOutput);
    bool AtTarget();
    void SetPosition(double position);
    double GetSetpoint();
    void ResetConstants(double upLimit, double downLimit, double peakOutput);

    double GetDegreesPerTick() { return m_DegreesPerTick; }

    void SetClosedLoopError(int error) { m_MotorA->SetClosedLoopError(error); }

    void DisableOutput(bool disable) {}

    void DisabledCalibration();
    void handle();

    double GetPosition();

    void SetBrakeMode()
    {
        m_MotorA->SetNeutralMode(CowLib::CowMotorController::BRAKE);
        m_MotorB->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    }

    void SetCoastMode()
    {
        m_MotorA->SetNeutralMode(CowLib::CowMotorController::COAST);
        m_MotorB->SetNeutralMode(CowLib::CowMotorController::COAST);
    }

    void SetCurrentLimit(double peakAmps, double continuousAmps, int peakDuration, int ms);
    virtual ~Arm();

private:
    CowLib::CowMotorController *m_MotorA;
    CowLib::CowMotorController *m_MotorB;
    int m_MotorAID;
    double m_Position;
    double m_UpLimit;
    double m_DownLimit;
    double m_PeakOutput;
    double m_DegreesPerTick;
    bool m_CalculateGain;
    std::string m_Name;
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */
