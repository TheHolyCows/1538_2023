#ifndef __COWLIB_COW_SWERVE_MODULE_POSITION_H__
#define __COWLIB_COW_SWERVE_MODULE_POSITION_H__

#include <frc/kinematics/SwerveModulePosition.h>

namespace CowLib
{

    struct CowSwerveModulePosition
    {
        double distance;
        double angle;

        static CowSwerveModulePosition FromWPI(frc::SwerveModulePosition state)
        {
            return CowSwerveModulePosition{ state.distance.convert<units::feet>().value(),
                                            state.angle.Degrees().value() };
        }

        frc::SwerveModulePosition ToWPI()
        {
            return frc::SwerveModulePosition{ units::foot_t(distance).convert<units::meter>(),
                                              frc::Rotation2d{ units::degree_t(angle) } };
        }
    };

} // namespace CowLib

#endif /* __COWLIB_COW_SWERVE_MODULE_POSITION_H__ */
