//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowLogger.h
// author: jon-bassi
// created on: 2022-1-16
//==================================================

#ifndef __COW_LOGGER_H__
#define __COW_LOGGER_H__

#include "../CowConstants.h"

#include <errno.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <string>

#include <arpa/inet.h>
#include <fstream>
#include <mutex>
#include <queue>
#include <sys/socket.h>
#include <thread>

namespace CowLib
{

    class CowLogger
    {
    public:
        static CowLogger *GetInstance();
        ~CowLogger();

        enum CowLogMsgType : uint16_t
        {
            MSG_LOG = 0,
            PID_LOG,
            MOTR_LOG,
            BATT_LOG
        };

        enum CowLogLevel : uint16_t
        {
            LOG_OFF = 0,
            LOG_ERR,
            LOG_WARN,
            LOG_DBG,
            LOG_INFO
        };

        static void LogMsg(CowLogLevel,const char*);
        static void LogPID(uint32_t, double, double, double, double, double);
        static void LogMotor(uint32_t, double, double);

    private:
        CowLogger();
        static int SendLog(void*,size_t);
        static CowLogger *m_Instance;
        struct sockaddr_in m_LogServer;
        int m_LogSocket;

        const char *m_LogServerIP = "10.15.38.200";
        uint16_t m_LogServerPort  = 5810;

        struct CowLogHdr
        {
            CowLogMsgType msgType;
            uint16_t msgLen;
        };

        struct CowMsgLog
        {
            CowLogHdr hdr;
            CowLogLevel logLevel;
            char logStr[256];
        };

        struct CowPIDLog
        {
            CowLogHdr hdr;
            uint32_t motorId;
            double setPoint;
            double procVar;
            double pVar;
            double iVar;
            double dVar;
        };

        struct CowMotorLog
        {
            CowLogHdr hdr;
            uint32_t motorId;
            double temp;
            double encoderCt; 
        };

        struct CowBattLog
        {
            CowLogHdr hdr;
            double voltage;
        };
    };

} /* namespace CowLib */

#endif /* __COW_LOGGER_H__ */
