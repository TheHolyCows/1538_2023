/*
 * CowLatch.h
 *
 *  Created on: Feb 20, 2016
 *      Author: kchau
 */

#ifndef __COWLIB_COWLATCH_H__
#define __COWLIB_COWLATCH_H__

namespace CowLib
{

    class CowLatch
    {
    private:
        bool m_State;
        bool m_Latched;

    public:
        CowLatch();
        bool Latch(bool value);
        void ResetLatch();
        virtual ~CowLatch();
    };

} /* namespace CowLib */

#endif /* __COWLIB_COWLATCH_H__ */
