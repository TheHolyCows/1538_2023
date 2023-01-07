//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================
#ifndef __COWLIB_COWCIRCULARBUFFER_H__
#define __COWLIB_COWCIRCULARBUFFER_H__

#include <stdint.h>

namespace CowLib
{
    class CowCircularBuffer
    {
    public:
        CowCircularBuffer(uint32_t sizeInBytes, bool overwriteWhenFull);
        virtual ~CowCircularBuffer();
        bool IsEmpty();
        bool IsFull();
        void Clear();
        uint32_t GetUsed();
        uint32_t GetRemaining();
        uint32_t GetSize();
        uint32_t GetBuffer(void *buf, uint32_t size);
        uint32_t PutBuffer(void *buf, uint32_t size);

    private:
        char *m_Buffer;
        uint32_t m_Size;
        uint32_t m_StartIndex;
        uint32_t m_EndIndex;
        bool m_OverwriteWhenFull;
    };
} // namespace CowLib

#endif // __COWLIB_COWCIRCULARBUFFER_H__
