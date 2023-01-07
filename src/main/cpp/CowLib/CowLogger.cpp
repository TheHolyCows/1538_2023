//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowLogger.h
// author: jon-bassi
// created on: 2022-1-16
//==================================================

#include "CowLogger.h"

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
        m_LogSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        if (m_LogSocket < 0)
        {
            std::cout << "error creating socket" << std::endl;
        }

        m_LogServer.sin_family      = AF_INET;
        m_LogServer.sin_addr.s_addr = inet_addr(m_LogServerIP);
        m_LogServer.sin_port        = htons(m_LogServerPort);
    }

    /**
     * CowLogger::SendLog
     * handles common send routine of log packets
     * @arg buf - logPacket
     * @arg bufSize - sizeof(logPacket) 
    */
    int CowLogger::SendLog(void* buf, size_t bufSize)
    {
        int ret = sendto(GetInstance()->m_LogSocket,
                         buf,
                         bufSize,
                         0,
                         reinterpret_cast<sockaddr *>(&GetInstance()->m_LogServer),
                         sizeof(m_LogServer));
        if (ret == -1)
        {
            std::cout << "CowLogger::SendLog() errno: " << strerror(errno) << std::endl;
        }

        return ret;
    }

    /**
     * CowLogger::LogMsg
     * sends a message with log level to our log server
     * message is limited to 255 characters
     * e.x. CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_INFO,"CowRobot initialized");
     * @arg logLevel - enum specified in this header
     * @arg logStr - string to send to log server
    */
   void CowLogger::LogMsg(CowLogLevel logLevel, const char* logStr)
   {
        if (CONSTANT("DEBUG_PID") == 0)
        {
            return;
        }
        
        CowMsgLog logPacket;
        logPacket.hdr.msgType = CowLogMsgType::MSG_LOG;
        logPacket.hdr.msgLen = sizeof(logPacket);
        logPacket.logLevel = logLevel;
        // sub 1 from len to allow space for '\0'
        memset(logPacket.logStr,0x0,sizeof(logPacket.logStr));
        strncpy(logPacket.logStr,logStr,sizeof(logPacket.logStr) - 1);

        // we are not checking to see if the strlen is less than 255 and therefore
        // sending the full packet each time, for efficiency we should only send 
        // bytes that are utilized + 1 null byte
        SendLog(&logPacket,sizeof(logPacket));
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

        SendLog(&logPacket,sizeof(logPacket));
    }

    /**
     * CowLogger::LogMotor
     * sends a selection of motor information to log server
     * @arg motorId - id of the motor
     * @arg temp - temperatur of the motor in celcius
     * @arg encoderCt - current encoder count of the motor
    */
    void CowLogger::LogMotor(uint32_t motorId, double temp, double encoderCt)
    {
        if (CONSTANT("DEBUG_PID") == 0)
        {
            return;
        }

        CowMotorLog logPacket;

        logPacket.hdr.msgType = CowLogger::MOTR_LOG;
        logPacket.hdr.msgLen = sizeof(CowMotorLog);
        logPacket.motorId = motorId;
        logPacket.temp = temp;
        logPacket.encoderCt = encoderCt;

        SendLog(&logPacket,sizeof(logPacket));
    }

} /* namespace CowLib */
