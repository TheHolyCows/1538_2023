#include "CowMotorController.h"

#include "CowLogger.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"

namespace CowLib
{

    CowMotorController::CowMotorController(int id)
    {
        m_Talon = new ctre::phoenixpro::hardware::TalonFX(id, "cowbus");
        CowLogger::GetInstance()->RegisterMotor(id, this);
    }

    CowMotorController::~CowMotorController()
    {
        delete m_Talon;
    }

    ctre::phoenixpro::hardware::TalonFX *CowMotorController::GetInternalTalon()
    {
        return m_Talon;
    }

    ctre::phoenixpro::controls::ControlRequest *CowMotorController::TranslateControlMode(ControlMode mode,
                                                                                         ControlMethod method,
                                                                                         double value,
                                                                                         double feedForward,
                                                                                         bool useFOC,
                                                                                         bool overrideBrakeMode)
    {
        switch (mode)
        {
        case OUT :
            if (method == DUTY_CYCLE)
            {
                return new ctre::phoenixpro::controls::DutyCycleOut(value, useFOC, overrideBrakeMode);
            }
            else if (method == VOLTAGE)
            {
                return new ctre::phoenixpro::controls::VoltageOut(units::volt_t{ value }, useFOC, overrideBrakeMode);
            }
            else if (method == TORQUE)
            {
                return new ctre::phoenixpro::controls::TorqueCurrentFOC(units::ampere_t{ 0_A },
                                                                        1.0,
                                                                        0_A,
                                                                        overrideBrakeMode);
            }
            else
            {
                return nullptr;
            }

        case POSITION :
        {
            auto position = units::turn_t{ value };

            if (method == DUTY_CYCLE)
            {
                return new ctre::phoenixpro::controls::PositionDutyCycle(position, useFOC, 0, 0, overrideBrakeMode);
            }
            else if (method == VOLTAGE)
            {
                return new ctre::phoenixpro::controls::PositionVoltage(position, useFOC, 0_V, 0, overrideBrakeMode);
            }
            else if (method == TORQUE)
            {
                return new ctre::phoenixpro::controls::PositionTorqueCurrentFOC(position, 0_A, 0, overrideBrakeMode);
            }
            else
            {
                return nullptr;
            }
        }
        case VELOCITY :
        {
            auto velocity = units::turns_per_second_t{ value };

            if (method == DUTY_CYCLE)
            {
                return new ctre::phoenixpro::controls::VelocityDutyCycle(velocity, useFOC, 0, 0, overrideBrakeMode);
            }
            else if (method == VOLTAGE)
            {
                return new ctre::phoenixpro::controls::VelocityVoltage(velocity, useFOC, 0_V, 0, overrideBrakeMode);
            }
            else if (method == TORQUE)
            {
                return new ctre::phoenixpro::controls::VelocityTorqueCurrentFOC(velocity, 0_A, 0, overrideBrakeMode);
            }
            else
            {
                return nullptr;
            }
        }
        case MOTION_MAGIC :
        {
            auto position = units::turn_t{ value };

            if (method == DUTY_CYCLE)
            {
                return new ctre::phoenixpro::controls::MotionMagicDutyCycle(position, useFOC, 0, 0, overrideBrakeMode);
            }
            else if (method == VOLTAGE)
            {
                return new ctre::phoenixpro::controls::MotionMagicVoltage(position, useFOC, 0_V, 0, overrideBrakeMode);
            }
            else if (method == TORQUE)
            {
                return new ctre::phoenixpro::controls::MotionMagicTorqueCurrentFOC(position, 0, 0, overrideBrakeMode);
            }
            else
            {
                return nullptr;
            }
        }
        case FOLLOWER :
            return new ctre::phoenixpro::controls::Follower(value, false);
        case NO_CONTROL_MODE :
            return nullptr;
        default :
            return nullptr;
        }
    }

    void CowMotorController::SetControlMode(ControlMode mode)
    {
        auto newCtrl = TranslateControlMode(mode, m_ControlMethod, 0, 0, true, false);
        if (newCtrl == nullptr)
        {
            return;
        }

        m_ControlMode = mode;
        if (typeid(*m_ControlRequest) != typeid(*newCtrl))
        {
            m_ControlRequest = newCtrl;
        }
    };

    void CowMotorController::Set(double value)
    {
        // switch (m_ControlMethod)
        // {
        // case DUTY_CYCLE :)
        //     break;
        // case VOLTAGE :
        //     break;
        // case TORQUE :
        //     break;
        // default :
        //     break;
        // }
    }

    void CowMotorController::SetControlMethod(ControlMethod method){

    };

    void CowMotorController::ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                                      ctre::phoenixpro::configs::Slot0Configs,
                                                      ctre::phoenixpro::configs::MotionMagicConfigs> config)
    {
        auto &configuator = m_Talon->GetConfigurator();

        visit([&configuator](auto &&config) { configuator.Apply(config); }, config);
    }

    void CowMotorController::SetControl(ctre::phoenixpro::controls::ControlRequest &control)
    {
        m_Talon->SetControl(control);
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

    void CowMotorController::SetInverted(bool inverted)
    {
        m_Talon->SetInverted(inverted);
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

    void CowMotorController::GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D)
    {
        *setpoint = -1;
        *procVar  = GetPosition();
        *P        = -1;
        *I        = -1;
        *D        = -1;
    }

    void CowMotorController::GetLogData(double *temp, double *encoderCt, bool *isInverted)
    {
        *encoderCt = GetPosition();
    }

} // namespace CowLib
