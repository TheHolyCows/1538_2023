//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWALPHANUM_H__
#define __COWLIB_COWALPHANUM_H__

#include <frc/I2C.h>
#include <stdint.h>
#include <string.h>
#include <string>

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ 1
#define HT16K33_BLINK_1HZ 2
#define HT16K33_BLINK_HALFHZ 3
#define HT16K33_OSCILLATOR 0x21
#define HT16K33_CMD_BRIGHTNESS 0xE0
#define HT16K33_CMD_BUFFER_SIZE 17

namespace CowLib
{
    class CowAlphaNum
    {
    public:
        CowAlphaNum(uint8_t address);
        virtual ~CowAlphaNum();
        void BlinkRate(uint8_t b);
        void SetBrightness(uint8_t b);
        void WriteAscii(uint32_t n, uint8_t c, bool d);
        void WriteRaw(uint32_t n, uint16_t d);
        void Clear();
        void Display();
        void SetBanner(std::string);
        void SetBannerPosition(uint32_t);
        void DisplayBanner();
        void OscillatorOn();

    private:
        uint8_t m_Address;
        frc::I2C *m_I2C;
        uint16_t *m_Banner;
        uint32_t m_BannerLength;
        uint32_t m_BannerPosition;
        uint8_t m_DisplayBuffer[HT16K33_CMD_BUFFER_SIZE];
    };
} // namespace CowLib

#endif /* __COWLIB_COWALPHANUM_H__ */
