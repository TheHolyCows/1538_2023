#include "CowMotorController.h"

#include "CowLogger.h"

#include <utility>

namespace CowLib
{
    /** 
     * @brief Construct a new Cow Motor Controller
     * @param id The CAN ID of the motor controller
     */
    CowMotorController::CowMotorController(int id, std::string bus)
    {
        m_Talon             = new ctre::phoenixpro::hardware::TalonFX(id, std::move(bus));
        m_Setpoint          = 0;
        m_UseFOC            = true;
        m_OverrideBrakeMode = false;

        ApplyConfig(ctre::phoenixpro::configs::TalonFXConfiguration{});

        CowLogger::GetInstance()->RegisterMotor(id, this);
    }

    CowMotorController::~CowMotorController()
    {
        delete m_Talon;
    }

    /**
     * @brief Set the motor controller with a control request struct
     * @param request The control request struct
     */
    void CowMotorController::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
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

    // Overload for TorqueControl requests because they always use FOC
    void CowMotorController::Set(
        std::variant<TorqueCurrentOutput, PositionTorqueCurrent, VelocityTorqueCurrent, MotionMagicTorqueCurrent>
            request)
    {
        auto &talon      = m_Talon;
        double *setpoint = &m_Setpoint;
        visit(
            [talon, setpoint](auto &&req)
            {
                talon->SetControl(req.ToControlRequest());
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    // Overload for follwer request because it's special
    void CowMotorController::Set(Follower request)
    {
        m_Talon->SetControl(request.ToControlRequest());
        m_Setpoint = request.LeaderID;
    }

    /** 
     * @brief Enables or disables the use of Field Oriented Control, default is true
     */
    void CowMotorController::UseFOC(bool useFOC)
    {
        m_UseFOC = useFOC;
    }

    /**
     * @brief If overrided, seting an output of zero forces to motor to brake, default is false
     */
    void CowMotorController::OverrideBrakeMode(bool overrideBrakeMode)
    {
        m_OverrideBrakeMode = overrideBrakeMode;
    }

    /**
     * @brief Applies a config to the motor controller
     */
    void CowMotorController::ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                                      ctre::phoenixpro::configs::Slot0Configs,
                                                      ctre::phoenixpro::configs::MotionMagicConfigs,
                                                      ctre::phoenixpro::configs::MotorOutputConfigs> config)
    {
        auto &configuator = m_Talon->GetConfigurator();

        visit(
            [&configuator](auto &&config)
            {
                ctre::phoenix::StatusCode res;
                // do
                // {
                res = configuator.Apply(config);
                // } while (!res.IsOK());
            },
            config);
    }

    /** 
     * @brief Gets the current position of the motor
     * @return The position in turns
     */
    double CowMotorController::GetPosition()
    {
        return m_Talon->GetPosition().Refresh().GetValue().value();
    }

    /** 
     * @brief Gets the current velocity of the motor
     * @return The velocity in turns per second
     */
    double CowMotorController::GetVelocity()
    {
        return m_Talon->GetVelocity().Refresh().GetValue().value();
    }

    /** 
     * @brief Gets the cached current corresponding to the torque output by the motor
     * this is split from the refresh current getter due to the possibility of flooding
     * the CANBUS or the motor with requests
     * @return The current in amps
     */
    double CowMotorController::GetTorqueCurrent()
    {
        return m_Talon->GetTorqueCurrent().GetValue().value();
    }

    /** 
     * @brief Gets the updated current corresponding to the torque output by the motor
     * this is split from the standard current getter due to the possibility of flooding
     * the CANBUS or the motor with requests
     * @return The current in amps
     */
    double CowMotorController::GetRefreshTorqueCurrent()
    {
        return m_Talon->GetTorqueCurrent().Refresh().GetValue().value();
    }

    /** 
     * @brief Sets the current position of the motor to a new value in turns. Used to zero.
     * @return Status code returned by talon
     */
    int CowMotorController::SetSensorPosition(double turns)
    {
        return m_Talon->SetRotorPosition(units::turn_t{ turns });
    }

    void CowMotorController::SetNeutralMode(NeutralMode mode)
    {
        auto config = ctre::phoenixpro::configs::MotorOutputConfigs{};
        m_Talon->GetConfigurator().Refresh(config);

        switch (mode)
        {
        case COAST :
            config.NeutralMode = ctre::phoenixpro::signals::NeutralModeValue::Coast;
            break;
        case BRAKE :
            config.NeutralMode = ctre::phoenixpro::signals::NeutralModeValue::Brake;
            break;
        default :
            break;
        }

        auto res = m_Talon->GetConfigurator().Apply(config);
        // printf("neutral mode %s\n", res.GetName());

        // ApplyConfig(config);
    }

    CowMotorController::NeutralMode CowMotorController::GetNeutralMode()
    {
        auto config = ctre::phoenixpro::configs::MotorOutputConfigs{};
        m_Talon->GetConfigurator().Refresh(config);

        switch (config.NeutralMode.value)
        {
        case ctre::phoenixpro::signals::NeutralModeValue::Coast :
            return COAST;
        case ctre::phoenixpro::signals::NeutralModeValue::Brake :
            return BRAKE;
        default :
            return COAST;
        }
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
