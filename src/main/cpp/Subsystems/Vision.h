#ifndef __VISION_H__
#define __VISION_H__

#include "../CowConstants.h"
#include "../CowLib/CowLogger.h"
#include "../CowLib/CowPID.h"
#include "frc/controller/PIDController.h"
#include "Limelight.h"

#include <memory>

class Vision
{
public:
    enum GamePiece
    {
        CUBE,
        CONE
    };

private:
    static Vision *s_Instance;

    Vision();

    std::unique_ptr<CowLib::CowPID> m_ScoringYPID;
    std::unique_ptr<CowLib::CowPID> m_ScoringYawPID;
    frc2::PIDController ypid2;

public:
    static Vision *GetInstance();

    ~Vision() = default;

    /**
     * @brief Resets PID constants and controllers
     */
    void Reset();

    double ScoringYPID(GamePiece type);
    double ScoringYawPID();
};

#endif /* __VISION_H__ */