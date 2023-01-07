#include "CowMotorController.h"

#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"

namespace CowLib
{
    CowMotorController::CowMotorController(int deviceNum)
        : m_DeviceNum(deviceNum)
    {
        m_CowControlMode  = CowMotorController::PERCENTVBUS;
        m_CowNeutralMode  = CowMotorController::COAST;
        m_MotorController = new TalonFX(deviceNum);
    }

    CowMotorController::~CowMotorController()
    {
        delete m_MotorController;
    }

    void CowMotorController::SetNeutralMode(CowMotorController::CowNeutralMode mode)
    {
        m_CowNeutralMode = mode;
        switch (mode)
        {
        case CowMotorController::JUMPER :
            m_MotorController->SetNeutralMode(EEPROMSetting);
            break;
        case CowMotorController::BRAKE :
            m_MotorController->SetNeutralMode(Brake);
            break;
        case CowMotorController::COAST :
            m_MotorController->SetNeutralMode(Coast);
            break;
        default :
            // What?
            break;
        }
    }

    void CowMotorController::SetControlMode(CowMotorController::CowControlMode mode)
    {
        m_CowControlMode = mode;
    }

    ControlMode CowMotorController::TranslateControlMode(enum CowMotorController::CowControlMode mode)
    {
        ControlMode retVal = ControlMode::Disabled;

        switch (mode)
        {
        case CowMotorController::PERCENTVBUS :
            retVal = ControlMode::PercentOutput;
            break;
        case CowMotorController::CURRENT :
            retVal = ControlMode::Current;
            break;
        case CowMotorController::SPEED :
            retVal = ControlMode::Velocity;
            break;
        case CowMotorController::POSITION :
            retVal = ControlMode::Position;
            break;
        case CowMotorController::VOLTAGE :
            // Unsupported
            break;
        case CowMotorController::FOLLOWER :
            retVal = ControlMode::Follower;
            break;
        case CowMotorController::MOTIONPROFILE :
            retVal = ControlMode::MotionProfile;
            break;
        case CowMotorController::MOTIONMAGIC :
            retVal = ControlMode::MotionMagic;
            break;
        default :
            // What?
            break;
        }

        return retVal;
    }

    enum CowMotorController::CowControlMode CowMotorController::GetControlMode()
    {
        return m_CowControlMode;
    }

    double CowMotorController::GetPosition()
    {
        return m_MotorController->GetSelectedSensorPosition(0);
    }

    void CowMotorController::SetSensorPosition(double position)
    {
        m_MotorController->SetSelectedSensorPosition(position);
    }

    void CowMotorController::SetPIDGains(double pGain, double iGain, double dGain, double fGain, double peakOutput)
    {
        m_MotorController->Config_kP(0, pGain, 100);
        m_MotorController->Config_kI(0, iGain, 100);
        m_MotorController->Config_kD(0, dGain, 100);
        m_MotorController->Config_kF(0, fGain, 100);
        m_MotorController->ConfigPeakOutputForward(peakOutput);
        m_MotorController->ConfigPeakOutputReverse(-peakOutput);
    }

    void CowMotorController::SetMotionMagic(double accel, double velocity)
    {
        m_MotorController->ConfigMotionAcceleration(accel, 10);
        m_MotorController->ConfigMotionCruiseVelocity(velocity, 10);
    }

    void CowMotorController::SetPeakCurrent(int amps, int ms)
    {
        // i'm pretty sure this is deprecated on the FX in favor of StatorLimiting
        // m_MotorController->ConfigContinuousCurrentLimit(0);
        // m_MotorController->ConfigPeakCurrentLimit(amps);
        // m_MotorController->ConfigPeakCurrentDuration(ms);
        // m_MotorController->EnableCurrentLimit(true);
    }

    void CowMotorController::SetStatorLimit(double limit, double threshold, double duration)
    {
        m_MotorController->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration(true, limit, threshold, duration));
    }

    double CowMotorController::GetOutputCurrent()
    {
        return m_MotorController->GetOutputCurrent();
    }

    void CowMotorController::Set(double value)
    {
        m_MotorController->Set(TranslateControlMode(GetControlMode()), value);
    }

    void CowMotorController::SetInverted(bool Value)
    {
        // m_MotorController->SetSensorPhase(Value);
        m_MotorController->SetInverted(Value);
    }

    TalonFX *CowMotorController::GetInternalMotor()
    {
        return m_MotorController;
    }

    void CowMotorController::SetClosedLoopError(int error)
    {
        m_MotorController->ConfigAllowableClosedloopError(0, error);
    }
} // namespace CowLib
