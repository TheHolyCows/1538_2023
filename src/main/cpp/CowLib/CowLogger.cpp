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
        m_TickCount = 0;
        memset(m_RegisteredMotors, 0x0, sizeof(m_RegisteredMotors));

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
     * @param buf - logPacket
     * @param bufSize - sizeof(logPacket)
     */
    int CowLogger::SendLog(void *buf, size_t bufSize)
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
     * CowLogger::RegisterMotor
     * saves motor info to an array for use by the motor and PID debug logging functions
     */
    void CowLogger::RegisterMotor(uint32_t motorId, CowLib::CowMotorController *motorController)
    {
        if (m_Instance == NULL)
        {
            // will initialize incase we get here without setting things up
            GetInstance();
        }

        if (motorId >= 0 && motorId < REGISTERED_MOTORS_MAX && m_RegisteredMotors[motorId] == NULL)
        {
            m_RegisteredMotors[motorId] = motorController;
        }
        else
        {
            std::cout << "CowLogger::RegisterMotor() error: motorID: " << motorId << " has already been registered"
                      << std::endl;
        }
    }

    /**
     * CowLogger::LogMsg
     * sends a message with log level to our log server
     * message is limited to 255 characters
     * e.x. CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_INFO,"CowRobot initialized");
     * @param logLevel - enum specified in this header
     * @param logStr - string to send to log server
     */
    void CowLogger::LogMsg(CowLogLevel logLevel, const char *logStr)
    {
        if (CONSTANT("DEBUG") == 0)
        {
            return;
        }

        CowMsgLog logPacket;
        logPacket.hdr.msgType = CowLogMsgType::MSG_LOG;
        logPacket.hdr.msgLen  = sizeof(logPacket);
        logPacket.logLevel    = logLevel;
        // sub 1 from len to allow space for '\0'
        memset(logPacket.logStr, 0x0, sizeof(logPacket.logStr));
        strncpy(logPacket.logStr, logStr, sizeof(logPacket.logStr) - 1);

        // we are not checking to see if the strlen is less than 255 and therefore
        // sending the full packet each time, for efficiency we should only send
        // bytes that are utilized + 1 null byte
        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::LogPID(uint32_t motorId, double setPoint, double procVar, double P, double I, double D)
     * logs PID values to remote logging server for graphing and testing
     * @param setPoint - current value motor is attempting to reach
     * @param procVar - current motor speed in RPM (motor is 2048 units per revolution)
     * @param P
     * @param I
     * @param D
     **/
    void CowLogger::LogPID(uint32_t motorId, double setPoint, double procVar, double P, double I, double D)
    {
        if (CONSTANT("DEBUG") == 0)
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

        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::LogMotor
     * sends a selection of motor information to log server
     * @param motorId - id of the motor
     * @param temp - temperatur of the motor in celcius
     * @param encoderCt - current encoder count of the motor
     */
    void CowLogger::LogMotor(uint32_t motorId, double temp, double encoderCt)
    {
        if (CONSTANT("DEBUG") == 0)
        {
            return;
        }

        CowMotorLog logPacket;

        logPacket.hdr.msgType = CowLogger::MOTR_LOG;
        logPacket.hdr.msgLen  = sizeof(CowMotorLog);
        logPacket.motorId     = motorId;
        logPacket.temp        = temp;
        logPacket.encoderCt   = encoderCt;

        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::Handle
     * handler to be called every cycle for logging within CowRobot, strictly used to log registered motors in debug
     * mode, therefore, this function can safely be removed from production code
     */
    void CowLogger::Handle()
    {
        if (CONSTANT("DEBUG") == 0 || CONSTANT("DEBUG_MOTOR_ID") < 0
            || CONSTANT("DEBUG_MOTOR_ID") >= REGISTERED_MOTORS_MAX)
        {
            return;
        }

        int debugMotorID = CONSTANT("DEBUG_MOTOR_ID");

        if (m_Instance == NULL)
        {
            // will initialize incase we get here without setting things up
            GetInstance();
        }

        if (CONSTANT("DEBUG_MOTOR_PID") != 0) // every cycle
        {
            if (m_RegisteredMotors[debugMotorID] != NULL)
            {
                double setPoint;
                double procVar;
                double P;
                double I;
                double D;
                m_RegisteredMotors[debugMotorID]->GetPIDData(&setPoint, &procVar, &P, &I, &D);

                LogPID(debugMotorID, setPoint, procVar, P, I, D);
            }
        }

        if (m_TickCount % 50 == 0) // 500 miliseconds
        {
            if (m_RegisteredMotors[debugMotorID] != NULL)
            {
                double temp;
                double encoderCt;
                bool isInverted;
                m_RegisteredMotors[debugMotorID]->GetLogData(&temp, &encoderCt, &isInverted);

                LogMotor(debugMotorID, temp, encoderCt);
            }
        }
    }

} /* namespace CowLib */
