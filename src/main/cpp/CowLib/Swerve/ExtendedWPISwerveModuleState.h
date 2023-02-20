#pragma once

#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/velocity.h"

#include <units/angular_velocity.h>

namespace CowLib
{

    class ExtendedWPISwerveModuleState : public frc::SwerveModuleState
    {
    public:
        units::meters_per_second_t speed  = 0_mps;
        units::radians_per_second_t omega = 0_rad_per_s;
        frc::Rotation2d angle             = frc::Rotation2d{ 0_deg };

        ExtendedWPISwerveModuleState(units::meters_per_second_t speed_  = 0_mps,
                                     frc::Rotation2d angle_             = frc::Rotation2d{ 0_deg },
                                     units::radians_per_second_t omega_ = 0_rad_per_s)
        {
            speed = speed_;
            omega = omega_;
            angle = angle_;
        }
    };

} // namespace CowLib