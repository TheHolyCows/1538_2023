/*
 * CowLatch.cpp
 *
 *  Created on: Feb 20, 2016
 *      Author: kchau
 */

#include "CowLatch.h"

namespace CowLib
{

    CowLatch::CowLatch()
        : m_State(false),
          m_Latched(false)
    {
    }

    // Returns true if state has changed
    bool CowLatch::Latch(bool value)
    {
        bool stateChanged = false;
        if (!m_Latched && value)
        {
            m_State      = value;
            m_Latched    = true;
            stateChanged = true;
        }

        return stateChanged;
    }

    void CowLatch::ResetLatch()
    {
        m_State   = false;
        m_Latched = false;
    }

    CowLatch::~CowLatch()
    {
    }

} /* namespace CowLib */
