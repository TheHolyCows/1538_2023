#ifndef __COW_PIGEON_H__
#define __COW_PIGEON_H__

#include <ctre/phoenixpro/Pigeon2.hpp>

class CowPigeon
{
private:
    static CowPigeon *s_Instance;

    CowPigeon();
    ~CowPigeon() = default;

    ctre::phoenixpro::hardware::Pigeon2 *m_Pigeon;

    bool m_Inverted;

    double m_YawOffset;

public:
    static CowPigeon *GetInstance();

    void SetInverted(bool inverted);

    units::degree_t GetYaw();
    units::degree_t GetPitch();
    units::degree_t GetRoll();

    double GetYawDegrees();
    double GetPitchDegrees();
    double GetRollDegrees();

    void SetYaw(units::degree_t angle);
    void SetYaw(double angle);
};

#endif /* __COW_PIGEON_H__ */