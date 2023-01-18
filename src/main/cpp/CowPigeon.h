#ifndef __COW_PIGEON_H__
#define __COW_PIGEON_H__

#include <ctre/phoenix/sensors/Pigeon2.h>

class CowPigeon
{
private:
    static CowPigeon *s_Instance;

    CowPigeon();
    ~CowPigeon() = default;

    ctre::phoenix::sensors::Pigeon2 *m_Pigeon;

    bool m_Inverted;

    double m_YawOffset;

public:
    static CowPigeon *GetInstance();

    void SetInverted(bool inverted);

    double GetYaw();
    double GetPitch();
    double GetRoll();

    void SetYaw(double angle);
    // void SetPitch(double angle);
    // void SetRoll(double angle);

    double GetRawYaw();
    double GetRawPitch();
    double GetRawRoll();
};

#endif /* __COW_PIGEON_H__ */