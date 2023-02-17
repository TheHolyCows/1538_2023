#include "GenericInput.h"

GenericInput::GenericInput()
{

}

Button::Button(const int ID)
{
    
}
bool Button::OnPress()
{



    return m_Pressed;
}
bool Button::OnHold()
{

}

Trigger::Trigger(const int ID)
{

}
bool Trigger::OnPress()
{

}
bool Trigger::OnHold()
{

}
void Button::Handle()
{
    if(m_Value == false)
    {
        m_PressLocked = false;
    }
}

void Trigger::Handle()
{

}