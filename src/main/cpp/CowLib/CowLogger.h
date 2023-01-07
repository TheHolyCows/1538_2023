//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowLogger.h
// author: jon-bassi
// created on: 2022-1-16
//==================================================

#ifndef __COW_LOGGER_H__
#define __COW_LOGGER_H__

#include "../CowConstants.h"

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
        static void LogPID(uint32_t, double, double, double, double, double);

        enum CowLogMsgType : uint16_t
        {
            MSG_LOG = 0,
            PID_LOG,
            TEMP_LOG,
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

    private:
        CowLogger();
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
            char logStr[256]; // TODO: determine how to do this
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

        struct CowBattLog
        {
            CowLogHdr hdr;
            double voltage;
        };
    };

} /* namespace CowLib */

#endif /* __COW_LOGGER_H__ */
