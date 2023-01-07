//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowLogger.h
// author: jon-bassi
// created on: 2022-1-16
//==================================================

#include "CowLogger.h"

#include <errno.h>
#include <iostream>
#include <string.h>
#include <time.h>

namespace CowLib
{

    CowLogger *CowLogger::m_Instance = NULL;

    CowLogger *CowLogger::GetInstance()
    {
        if (m_Instance == NULL)
        {
            m_Instance = new CowLogger();
        }

        return m_Instance;
    }

    CowLogger::~CowLogger()
    {
        return;
    }

    CowLogger::CowLogger()
    {
        if (CONSTANT("DEBUG_PID") != 0)
        {
            m_LogSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

            if (m_LogSocket < 0)
            {
                std::cout << "error creating socket" << std::endl;
            }

            m_LogServer.sin_family      = AF_INET;
            m_LogServer.sin_addr.s_addr = inet_addr(m_LogServerIP);
            m_LogServer.sin_port        = htons(m_LogServerPort);
        }
        else
        {
            m_LogSocket = -1;
        }
    }

    /**
     * CowLogger::LogPID(uint32_t motorId, double setPoint, double procVar, double P, double I, double D)
     * logs PID values to remote logging server for graphing and testing
     * @arg motorId
     * @arg setPoint
     * @arg procVar
     * @arg P
     * @arg I
     * @arg D
     **/
    void CowLogger::LogPID(uint32_t motorId, double setPoint, double procVar, double P, double I, double D)
    {
        if (CONSTANT("DEBUG_PID") == 0)
        {
            return;
        }

        CowPIDLog logPacket;

        logPacket.hdr.msgType = CowLogMsgType::PID_LOG;
        logPacket.hdr.msgLen  = sizeof(CowPIDLog);
        logPacket.motorId     = motorId;
        logPacket.setPoint    = setPoint;
        logPacket.procVar     = procVar;
        logPacket.pVar        = P;
        logPacket.iVar        = I;
        logPacket.dVar        = D;

        int ret = sendto(GetInstance()->m_LogSocket,
                         &logPacket,
                         sizeof(CowPIDLog),
                         0,
                         reinterpret_cast<sockaddr *>(&GetInstance()->m_LogServer),
                         sizeof(m_LogServer));
        if (ret == -1)
        {
            std::cout << "CowLogger::LogPID() errno: " << strerror(errno) << std::endl;
        }
    }

} /* namespace CowLib */
