#pragma once /* __ARM_STATE_H__ */

enum ARM_STATE
{
    ARM_NONE = 0,
    ARM_IN,
    ARM_STOW,
    ARM_L3,
    ARM_L2,
    ARM_L1,
    ARM_SCORE,
    ARM_MANUAL
};

enum ARM_CARGO
{
    ST_NONE = 0,
    ST_CONE,
    ST_CUBE
};