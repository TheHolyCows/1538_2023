#pragma once /* __ARM_STATE_H__ */

enum ARM_STATE
{
    ARM_NONE = 0,
    ARM_IN,
    ARM_STOW,
    ARM_L3,
    ARM_L2,
    ARM_GND,
    ARM_SCORE,
    ARM_HUMAN,
    ARM_MANUAL,
};

enum ARM_CARGO
{
    CG_NONE = 0,
    CG_CONE,
    CG_CUBE
};