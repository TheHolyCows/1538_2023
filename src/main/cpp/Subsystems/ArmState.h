#pragma once /* __ARM_STATE_H__ */

enum ARM_STATE
{
    ARM_NONE = 0,
    ARM_STOW,
    ARM_L3,
    ARM_L2,
    ARM_GND,
    ARM_HUMAN,
    ARM_DRIVER_STOW,
    ARM_UP
};

enum CLAW_STATE
{
    CLAW_OFF = 0,
    CLAW_INTAKE,
    CLAW_EXHAUST,
    CLAW_NONE,
};

enum ARM_CARGO
{
    CG_NONE = 0,
    CG_CONE,
    CG_CUBE
};
