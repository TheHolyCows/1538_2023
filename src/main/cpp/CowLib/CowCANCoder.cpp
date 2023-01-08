#include "CowCANCoder.h"

namespace CowLib {

/// @brief Creates a new CowCANCoder. Defaults to unsigned absolute
/// @param deviceId
CowCANCoder::CowCANCoder(int deviceId)
{
    m_Cancoder = new CANCoder(deviceId);
    m_Cancoder->ConfigFactoryDefault();

    // Default to absolute and unsigned
    SetInitToAbsolute(true);
    SetSigned(false);

    // This is what 1678 uses for swerve so okay
    // TODO: check if this is correct
    m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 255);
    m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 255);
}

/// @brief Sets whether the sensor direction is inverted
/// @param isInverted
void CowCANCoder::SetInverted(bool isInverted)
{
    m_Cancoder->ConfigSensorDirection(isInverted);
}

/// @brief Sets whether the output is signed [-180, 180] or unsigned [0, 360]
/// @param isSigned
void CowCANCoder::SetSigned(bool isSigned)
{
    AbsoluteSensorRange range;

    if (isSigned) {
        range = Signed_PlusMinus180;
    } else {
        range = Unsigned_0_to_360;
    }

    m_Cancoder->ConfigAbsoluteSensorRange(range);
}

/// @brief Sets whether the CANCoder saves its absolute position on boot
/// @param isAbsolute
void CowCANCoder::SetInitToAbsolute(bool isAbsolute)
{
    SensorInitializationStrategy strategy;

    if (isAbsolute) {
        strategy = BootToAbsolutePosition;
    } else {
        strategy = BootToZero;
    }

    m_Cancoder->ConfigSensorInitializationStrategy(strategy);
}

/// @brief Gets the absolute position in degrees
/// @return rotation
double CowCANCoder::GetAbsolutePosition()
{
    return m_Cancoder->GetAbsolutePosition();
}

/// @brief Gets internal CANCoder
/// @return Pointer to internal CANCoder
CANCoder* CowCANCoder::GetInternalCANCoder()
{
    return m_Cancoder;
}

CowCANCoder::~CowCANCoder()
{
    delete m_Cancoder;
}

}