#ifndef __COWLIB_GENERICINPUT_H__
#define __COWLIB_GENERICINPUT_H__

#include "../CowConstants.h"

class GenericInput
{
    virtual void Handle()  = 0;
    virtual bool OnPress() = 0;
    virtual bool OnHold()  = 0;
};

class Button : public GenericInput
{
public:
    Button(const int ID);
    void Handle();
    bool OnPress();
    bool OnHold();

private:
    bool m_Value = false;
    bool m_PressLocked;
};

class Trigger : public GenericInput
{
public:
    Trigger(const int ID);
    void Handle();
    bool OnPress();
    bool OnHold();

private:
    double m_Value   = 0;
    bool m_Triggered = false;
    bool m_PressLocked;
};

#endif /* __COWLIB_GENERICINPUT_H__ */