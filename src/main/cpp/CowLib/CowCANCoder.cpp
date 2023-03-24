#include "CowCANCoder.h"

namespace CowLib
{

    /// @brief Creates a new CowCANCoder. Defaults to unsigned absolute
    /// @param deviceId
    CowCANCoder::CowCANCoder(int deviceId)
    {
        m_Cancoder = new ctre::phoenixpro::hardware::CANcoder(deviceId, "cowdrive");

        // Default config
        m_Config = ctre::phoenixpro::configs::CANcoderConfiguration{};
        ApplyConfig();

        // This is what 1678 uses for swerve so okay
        // TODO: check if this is correct
        // m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 255);
        // m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 255);

        m_PositionSupplier         = m_Cancoder->GetPosition().AsSupplier();
        m_AbsolutePositionSupplier = m_Cancoder->GetAbsolutePosition().AsSupplier();
        m_VelocitySupplier         = m_Cancoder->GetVelocity().AsSupplier();
    }

    void CowCANCoder::ApplyConfig()
    {
        m_Cancoder->GetConfigurator().Apply(m_Config);
    }

    /// @brief Sets whether the sensor direction is inverted
    /// @param isInverted
    void CowCANCoder::SetInverted(bool isInverted)
    {
        m_Config.MagnetSensor.SensorDirection
            = isInverted ? ctre::phoenixpro::signals::SensorDirectionValue::Clockwise_Positive
                         : ctre::phoenixpro::signals::SensorDirectionValue::CounterClockwise_Positive;
    }

    /// @brief Sets whether the output is signed [-180, 180] or unsigned [0, 360]
    /// @param isSigned
    void CowCANCoder::SetSigned(bool isSigned)
    {
        // auto conf = m_Cancoder->GetConfigurator();
        m_Config.MagnetSensor.AbsoluteSensorRange
            = isSigned ? ctre::phoenixpro::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf
                       : ctre::phoenixpro::signals::AbsoluteSensorRangeValue::Unsigned_0To1;

        ApplyConfig();
    }

    double CowCANCoder::GetPosition()
    {
        // m_Signals.positionRefresh();
        return m_PositionSupplier().value();
    }

    /// @brief Gets the absolute position in degrees
    /// @return rotation
    double CowCANCoder::GetAbsolutePosition()
    {
        // m_Signals.absolutePosition->Refresh();
        // return m_Signals.absolutePosition->GetValue().value();
        return m_AbsolutePositionSupplier().value() * 360;
    }

    double CowCANCoder::GetVelocity()
    {
        // m_Signals.velocity->Refresh();
        // return m_Signals.velocity->GetValue().value();
        return m_VelocitySupplier().value();
    }

    /// @brief Gets internal CANCoder
    /// @return Pointer to internal CANCoder
    ctre::phoenixpro::hardware::CANcoder *CowCANCoder::GetInternalCANCoder()
    {
        return m_Cancoder;
    }

    CowCANCoder::~CowCANCoder()
    {
        delete m_Cancoder;
    }

} // namespace CowLib