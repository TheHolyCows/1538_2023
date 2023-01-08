#ifndef __COWLIB_COWCANCODER_H__
#define __COWLIB_COWCANCODER_H__

#include <ctre/Phoenix.h>

namespace CowLib {

class CowCANCoder {
private:
    CANCoder* m_Cancoder;

public:
    CowCANCoder(int cancoder);

    void SetSigned(bool isSigned);
    void SetInverted(bool isInverted);
    void SetInitToAbsolute(bool isAbsolute);

    double GetAbsolutePosition();

    CANCoder* GetInternalCANCoder();

    ~CowCANCoder();
};

}

#endif /* __COWLIB_COWCANCODER_H__ */