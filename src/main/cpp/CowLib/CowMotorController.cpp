#include "CowMotorController.h"

#include "CowLogger.h"

namespace CowLib
{

    CowMotorController::CowMotorController(int id)
    {
        m_Talon             = new ctre::phoenixpro::hardware::TalonFX(id, "cowbus");
        m_Setpoint          = 0;
        m_UseFOC            = true;
        m_OverrideBrakeMode = false;

        CowLogger::GetInstance()->RegisterMotor(id, this);
    }

    CowMotorController::~CowMotorController()
    {
        delete m_Talon;
    }

    void CowMotorController::Set(std::variant<PositionPercentOutput, PercentOutput> request)
    {
        auto &talon            = m_Talon;
        double *setpoint       = &m_Setpoint;
        bool useFOC            = m_UseFOC;
        bool overrideBrakeMode = m_OverrideBrakeMode;
        visit(
            [talon, setpoint, useFOC, overrideBrakeMode](auto &&req)
            {
                talon->SetControl(
                    req.ToControlRequest().WithEnableFOC(useFOC).WithOverrideBrakeDurNeutral(overrideBrakeMode));
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    void CowMotorController::UseFOC(bool useFOC)
    {
        m_UseFOC = useFOC;
    }

    void CowMotorController::OverrideBrakeMode(bool overrideBrakeMode)
    {
        m_OverrideBrakeMode = overrideBrakeMode;
    }

    void CowMotorController::ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                                      ctre::phoenixpro::configs::Slot0Configs,
                                                      ctre::phoenixpro::configs::MotionMagicConfigs> config)
    {
        auto &configuator = m_Talon->GetConfigurator();

        visit([&configuator](auto &&config) { configuator.Apply(config); }, config);
    }

    double CowMotorController::GetPosition()
    {
        return m_Talon->GetPosition().Refresh().GetValue().value();
    }

    double CowMotorController::GetVelocity()
    {
        return m_Talon->GetVelocity().Refresh().GetValue().value();
    }

    int CowMotorController::SetSensorPosition(double turns)
    {
        return m_Talon->SetRotorPosition(units::turn_t{ turns });
    }

    void CowMotorController::SetPID(double p, double i, double d, double f)
    {
        auto config = ctre::phoenixpro::configs::Slot0Configs{};

        config.kP = p;
        config.kI = i;
        config.kD = d;
        config.kV = f;

        ApplyConfig(config);
    }

    void CowMotorController::SetMotionMagic(double velocity, double acceleration)
    {
        auto config = ctre::phoenixpro::configs::MotionMagicConfigs{};

        config.MotionMagicCruiseVelocity = velocity;
        config.MotionMagicAcceleration   = acceleration;

        ApplyConfig(config);
    }

    void CowMotorController::SetInverted(bool inverted)
    {
        m_Talon->SetInverted(inverted);
    }

    ctre::phoenixpro::hardware::TalonFX *CowMotorController::GetInternalTalon()
    {
        return m_Talon;
    }

    void CowMotorController::GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D)
    {
        *setpoint = m_Setpoint;
        *procVar  = GetPosition();
        *P        = -1;
        *I        = -1;
        *D        = -1;
    }

    void CowMotorController::GetLogData(double *temp, double *encoderCt, bool *isInverted)
    {
        *temp       = m_Talon->GetDeviceTemp().Refresh().GetValue().value();
        *encoderCt  = GetPosition();
        *isInverted = m_Talon->GetInverted();
    }

} // namespace CowLib
