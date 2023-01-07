#include "CowCircularBuffer.h"

#include <stdio.h>
#include <string.h>

namespace CowLib
{
    CowCircularBuffer::CowCircularBuffer(uint32_t sizeInBytes, bool overwriteWhenFull)
        : m_Size(sizeInBytes + 1),
          m_StartIndex(0),
          m_EndIndex(0),
          m_OverwriteWhenFull(overwriteWhenFull)
    {
        m_Buffer = new char[sizeInBytes + 1];
        if (!m_Buffer)
        {
            printf("Could not create buffer!");
        }
    }

    CowCircularBuffer::~CowCircularBuffer()
    {
        delete[] m_Buffer;
    }

    void CowCircularBuffer::Clear()
    {
        m_StartIndex = 0;
        m_EndIndex   = 0;
    }

    bool CowCircularBuffer::IsEmpty()
    {
        bool result = (m_StartIndex == m_EndIndex);
        return result;
    }

    bool CowCircularBuffer::IsFull()
    {
        bool result = (((m_EndIndex + 1) % m_Size) == m_StartIndex);
        return result;
    }

    uint32_t CowCircularBuffer::GetUsed()
    {
        uint32_t result = 0;
        if (m_StartIndex > m_EndIndex)
        {
            result = m_EndIndex + m_Size - m_StartIndex;
        }
        else
        {
            result = m_EndIndex - m_StartIndex;
        }

        return result;
    }

    uint32_t CowCircularBuffer::GetRemaining()
    {
        return GetSize() - GetUsed();
    }

    uint32_t CowCircularBuffer::GetSize()
    {
        return m_Size - 1;
    }

    uint32_t CowCircularBuffer::GetBuffer(void *buf, uint32_t size)
    {
        uint32_t numToGet = 0;
        numToGet          = (size < GetUsed()) ? size : GetUsed();

        if (!size)
        {
            return 0;
        }

        if ((m_EndIndex + numToGet) > m_Size)
        {
            memcpy((char *) buf, &m_Buffer[m_StartIndex], m_Size - m_StartIndex);
            memcpy(&((char *) buf)[m_Size - m_StartIndex], m_Buffer, numToGet - (m_Size - m_StartIndex));
        }
        else
        {
            memcpy(buf, &m_Buffer[m_StartIndex], numToGet);
        }

        m_StartIndex = (m_StartIndex + numToGet) % m_Size;
        return numToGet;
    }

    uint32_t CowCircularBuffer::PutBuffer(void *buf, uint32_t size)
    {
        uint32_t numToPut = 0;
        uint32_t top      = 0;
        uint32_t bottom   = 0;

        if ((size > GetSize()) || !size)
        {
            printf("Invalid size to put!");
            return numToPut;
        }

        if (m_OverwriteWhenFull)
        {
            // Overwrite anything with the new buffer
            numToPut = size;

            // Update the start marker if necessary
            if (numToPut > GetRemaining())
            {
                m_StartIndex = (m_StartIndex + numToPut) % m_Size;
            }
        }
        else
        {
            // Only fill up what's left
            numToPut = (GetRemaining() > size) ? size : GetRemaining();
        }

        // Determine if we have to wrap around
        // Top is the high end of the circular buffer
        // Nottom is the beginning of the circular buffer
        if ((m_EndIndex + numToPut) > m_Size)
        {
            top    = m_Size - m_EndIndex;
            bottom = numToPut - top;
        }
        else
        {
            top = size;
        }

        memcpy(&m_Buffer[m_EndIndex], buf, top);
        m_EndIndex += top;

        if (bottom)
        {
            memcpy(m_Buffer, &((char *) buf)[top], bottom);
            m_EndIndex = bottom;
        }

        return numToPut;
    }
} // namespace CowLib
