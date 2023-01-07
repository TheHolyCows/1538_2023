#include "CowCanifier.h"

namespace CowLib
{
    CowCanifier::CowCanifier(int deviceNum)
        : m_DeviceNum(deviceNum),
          m_R(0),
          m_G(0),
          m_B(0),
          m_FlashTimer(0)
    {
        // m_Canifier = new CANifier(m_DeviceNum);
    }

    void CowCanifier::SetLEDColor(int R, int G, int B)
    {
        m_R = R;
        m_G = G;
        m_B = B;
    }

    void CowCanifier::FlashColor(int R, int G, int B)
    {
        // if (m_FlashTimer++ > CONSTANT("BLINK_SPEED"))
        // {
        //     m_FlashTimer = -CONSTANT("BLINK_SPEED");
        //     SetLEDColor(0, 0, 0);
        // }
        // else if (m_FlashTimer > 0)
        // {
        //     SetLEDColor(R, G, B);
        // }
    }

    void CowCanifier::ColorWheelScroll()
    {
        //     if ((m_R | m_B | m_G) == 0)
        //     {
        //         SetLEDColor(255,0,0);
        //         m_ScrollIndex[1] = 1;
        //     }

        //     if (m_ScrollIndex[0] != 0)
        //     {
        //         if (m_ScrollIndex[0] > 0 && m_R >= 255)
        //         {
        //             // magenta
        //             m_R = 255;
        //             m_ScrollIndex[0] = 0;
        //             m_ScrollIndex[2] = -1;
        //         }
        //         else if (m_ScrollIndex[0] < 0 && m_R <= 0)
        //         {
        //             // green
        //             m_R = 0;
        //             m_ScrollIndex[0] = 0;
        //             m_ScrollIndex[2] = 1;
        //         }
        //         else
        //         {
        //             m_R += 15 * m_ScrollIndex[0];
        //         }
        //     }
        //     if (m_ScrollIndex[1] != 0)
        //     {
        //         if (m_ScrollIndex[1] > 0 && m_G >= 255)
        //         {
        //             // yellow
        //             m_G = 255;
        //             m_ScrollIndex[1] = 0;
        //             m_ScrollIndex[0] = -1;
        //         }
        //         else if (m_ScrollIndex[1] < 0 && m_G <= 0)
        //         {
        //             // blue
        //             m_G = 0;
        //             m_ScrollIndex[1] = 0;
        //             m_ScrollIndex[0] = 1;
        //         }
        //         else
        //         {
        //             m_G += 15 * m_ScrollIndex[1];
        //         }
        //     }
        //     if (m_ScrollIndex[2] != 0)
        //     {
        //         if (m_ScrollIndex[2] > 0 && m_B >= 255)
        //         {
        //             // cyan
        //             m_B = 255;
        //             m_ScrollIndex[2] = 0;
        //             m_ScrollIndex[1] = -1;
        //         }
        //         else if (m_ScrollIndex[2] < 0 && m_B <= 0)
        //         {
        //             // red
        //             m_B = 0;
        //             m_ScrollIndex[2] = 0;
        //             m_ScrollIndex[1] = 1;
        //         }
        //         else
        //         {
        //             m_B += 15 * m_ScrollIndex[2];
        //         }
        //     }
        // }

        // void CowCanifier::Handle()
        // {
        //     double dutyCycle_R = double(m_R) / 255.0;
        //     double dutyCycle_G = double(m_G) / 255.0;
        //     double dutyCycle_B = double(m_B) / 255.0;

        //     m_Canifier->SetLEDOutput(dutyCycle_R, ctre::phoenix::CANifier::LEDChannelB);
        //     m_Canifier->SetLEDOutput(dutyCycle_G, ctre::phoenix::CANifier::LEDChannelA);
        //     m_Canifier->SetLEDOutput(dutyCycle_B, ctre::phoenix::CANifier::LEDChannelC);
    }

    CowCanifier::~CowCanifier()
    {
        delete m_Canifier;
    }
} // namespace CowLib
