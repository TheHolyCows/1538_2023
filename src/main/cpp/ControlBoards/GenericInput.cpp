#include "GenericInput.h"

//  --  All Button Code  --
Button::Button(const int ID)
{
    m_ID = ID;
}

bool Button::OnPress()
{
    if (m_Value && !m_PressLocked)
    {
        m_PressLocked = true;
        return true;
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
    m_Value = m_Controller->GetRawButton(m_ID);

    if (!m_Value)
    {
        m_PressLocked = false;
    }
}

//  --  All Trigger Code  --
Trigger::Trigger(const int ID)
{
     m_ID = ID;
}

bool Trigger::OnPress()
{
    if (m_Triggered && !m_PressLocked)
    {   
        m_PressLocked = true;
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
    m_Value = m_Controller->GetRawAxis(m_ID);

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