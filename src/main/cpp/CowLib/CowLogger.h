//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowLogger.h
// author: jon-bassi
// created on: 2022-1-16
//==================================================

#ifndef __COW_LOGGER_H__
#define __COW_LOGGER_H__

#include "../CowConstants.h"
#include "CowMotorController.h"

#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <vector>

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
            BATT_LOG,
            AUTO_LOG,
            GYRO_LOG
        };

        enum CowLogLevel : uint16_t
        {
            LOG_OFF = 0,
            LOG_ERR,
            LOG_WARN,
            LOG_DBG,
            LOG_INFO
        };

        const static int REGISTERED_MOTORS_MAX = 24;

        void RegisterMotor(uint32_t, CowLib::CowMotorController *);
        static void LogAutoMode(const char *);
        static void LogGyroAngle(double);
        static void LogMsg(CowLogLevel, const char *);

        void Handle();

    private:
        CowLogger();
        static int SendLog(void *, size_t);
        static void LogPID(uint32_t, double, double, double, double, double);
        static void LogMotor(uint32_t, double, double);

        static CowLogger *m_Instance;
        struct sockaddr_in m_LogServer;
        int m_LogSocket;

        uint32_t m_TickCount;

        uint32_t m_IdToLog;

        const char *m_LogServerIP = "10.15.38.200";
        uint16_t m_LogServerPort  = 5810;

        // assuming we don't have more than 24 motors ever
        CowLib::CowMotorController *m_RegisteredMotors[REGISTERED_MOTORS_MAX];

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

        struct CowAutoLog
        {
            CowLogHdr hdr;
            char name[32];
        };

        struct CowGyroLog
        {
            CowLogHdr hdr;
            double angle;
        };
    };

} /* namespace CowLib */

#endif /* __COW_LOGGER_H__ */
