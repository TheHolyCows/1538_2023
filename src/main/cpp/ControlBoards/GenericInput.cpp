#include "GenericInput.h"

//  --  All Button Code  --
Button::Button(const int ID)
{
    // m_Value should be Linked with button input
}

bool Button::OnPress()
{
    if (m_Value && !m_PressLocked)
    {
        return true;
        m_PressLocked = true;
    }
    else
    {
        return false;
    }
}

bool Button::OnHold()
{
    return m_Value;
}

void Button::Handle()
{
    if (!m_Value)
    {
        m_PressLocked = false;
    }
}

//  --  All Trigger Code  --
Trigger::Trigger(const int ID)
{
    // m_Value should be linked with trigger % value
}

bool Trigger::OnPress()
{
    if (m_Triggered && !m_PressLocked)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Trigger::OnHold()
{
    if (m_Triggered)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Trigger::Handle()
{
    if (m_Value > CONSTANT("TRIGGER_SENSITIVITY"))
    {
        m_Triggered = true;
    }
    else
    {
        m_PressLocked = false;
        m_Triggered   = false;
    }
}