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
        m_IdToLog   = 0;
        memset(m_RegisteredMotors, 0x0, sizeof(m_RegisteredMotors));

        m_LogSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        if (m_LogSocket < 0)
        {
            std::cout << "CowLogger::CowLogger() error: failed to create socket" << std::endl;
            return;
        }

        ResetConstants();

        m_LogServer.sin_family      = AF_INET;
        m_LogServer.sin_addr.s_addr = m_LogServerIP;
        m_LogServer.sin_port        = m_LogServerPort;
    }

    /**
     * CowLogger::ResetConstants()
     * resets log ip and port after reading values from constants
    */
    void CowLogger::ResetConstants()
    {
        uint32_t ipOctet = ((uint32_t) CONSTANT("LOG_SERVER_OCTET")) & 0xff;
        m_LogServerIP    = htonl(m_LogServerSubnet | ipOctet);

        uint16_t port   = ((uint16_t) CONSTANT("LOG_SERVER_PORT"));
        m_LogServerPort = htons(port);
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

        if (motorId < REGISTERED_MOTORS_MAX && m_RegisteredMotors[motorId] == NULL)
        {
            m_RegisteredMotors[motorId] = motorController;
        }
        else
        {
            LogMsg(CowLogger::LOG_ERR,
                   "CowLogger::RegisterMotor() error: motorID: %i has already been registered",
                   motorId);
        }
    }

    /**
     * CowLogger::LogAutoMode
     * unlike other loggers, this does not depend on debug flag being set
     * sends a message containing the name of the currently selected auto mode, up to 32 characters (including '\0')
     * called on every auto mode selection button press and every 50ms in disabled periodic
     * @param name - name of current auto mode
     */
    void CowLogger::LogAutoMode(frc::DriverStation::Alliance alliance, const char *name)
    {
        CowAutoLog logPacket;
        logPacket.hdr.msgType = CowLogger::AUTO_LOG;
        logPacket.hdr.msgLen  = sizeof(logPacket);

        logPacket.alliance = alliance;

        // sub 1 from len to allow space for '\0'
        memset(logPacket.name, 0x0, sizeof(logPacket.name));
        strncpy(logPacket.name, name, sizeof(logPacket.name) - 1);

        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::LogAutoMode
     * log the gyro from the robot, these values are relative to the starting orientation
     * of the bot
     * this can be reset by auto mode
     * @param pitch - pitch of the bot (in degrees)
     * @param roll - roll of the bot (in degrees)
     * @param yaw - yaw of the bot (in degrees)
    */
    void CowLogger::LogGyro(CowPigeon *gyro)
    {
        if ((int) CONSTANT("DEBUG") != CowLogger::LOG_DBG)
        {
            return;
        }

        double pitch = gyro->GetPitchDegrees();
        double roll  = gyro->GetRollDegrees();
        double yaw   = gyro->GetYawDegrees();

        CowGyroLog logPacket;
        logPacket.hdr.msgType = CowLogger::GYRO_LOG;
        logPacket.hdr.msgLen  = sizeof(CowGyroLog);

        logPacket.pitch = pitch;
        logPacket.roll  = roll;
        logPacket.yaw   = yaw;

        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::LogPose
     * log the current Pose of the robot
     * this is analogous to the current position of the bot on the field
     * this can be reset by auto mode
     * @param x - x position of the bot on the field (forwards and back for field oriented)
     * @param y - y position of the bot on the field (left and right for field oriented)
     * @param rot - yaw of the bot
    */
    void CowLogger::LogPose(double x, double y, double rot)
    {
        if ((int) CONSTANT("DEBUG") != CowLogger::LOG_DBG)
        {
            return;
        }

        CowPoseLog logPacket;
        logPacket.hdr.msgType = CowLogger::POSE_LOG;
        logPacket.hdr.msgLen  = sizeof(CowPoseLog);

        logPacket.x   = x;
        logPacket.y   = y;
        logPacket.rot = rot;

        SendLog(&logPacket, sizeof(logPacket));
    }

    /**
     * CowLogger::LogMsg
     * sends a message with log level to our log server
     * message is limited to 255 characters
     * e.x. CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_INFO,"CowRobot initialized");
     * @param logLevel - enum specified in this header
     * @param logStr - string to send to log server
     */
    void CowLogger::LogMsg(CowLogLevel logLevel, const char *fmt, ...)
    {
        if (logLevel > (int) CONSTANT("DEBUG"))
        {
            return;
        }

        CowMsgLog logPacket;
        logPacket.hdr.msgType = CowLogMsgType::MSG_LOG;
        logPacket.hdr.msgLen  = sizeof(logPacket);
        logPacket.logLevel    = logLevel;

        // pull all strings into logStr until no more strings or no more room
        va_list args;
        va_start(args, fmt);

        memset(logPacket.logStr, 0x0, sizeof(logPacket.logStr));
        // sub 1 from len to allow space for '\0'
        vsnprintf(logPacket.logStr, sizeof(logPacket.logStr) - 1, fmt, args);

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
        if ((int) CONSTANT("DEBUG") != CowLogger::LOG_DBG)
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
     * @param motorId id of the motor
     * @param temp - temperatur of the motor in celcius
     * @param encoderCt - current count of the motor
     */
    void CowLogger::LogMotor(uint32_t motorId, double temp, double encoderCt)
    {
        if ((int) CONSTANT("DEBUG") != CowLogger::LOG_DBG)
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
     * handler to be called every cycle for logging within CowRobot, strictly used
     * to log registered motors, therefore, this function can safely
     * be removed from production code
     */
    void CowLogger::Handle()
    {
        if ((int) CONSTANT("DEBUG") != CowLogger::LOG_DBG)
        {
            return;
        }

        if (m_Instance == NULL)
        {
            // will initialize incase we get here without setting things up
            GetInstance();
        }

        // if (CONSTANT("DEBUG_MOTOR_PID") >= 0) // every cycle
        // {
        //     int debugMotorID = CONSTANT("DEBUG_MOTOR_PID");
        //     if (m_RegisteredMotors[debugMotorID] != NULL)
        //     {
        //         double setPoint;
        //         double procVar;
        //         double P;
        //         double I;
        //         double D;
        //         m_RegisteredMotors[debugMotorID]->GetPIDData(&setPoint, &procVar, &P, &I, &D);

        //         LogPID(debugMotorID, setPoint, procVar, P, I, D);
        //     }
        // }

        if (m_TickCount++ % 10 == 0) // 200 miliseconds
        {
            m_TickCount = 1;

            uint32_t logsThisTick = 2;
            while (m_IdToLog < REGISTERED_MOTORS_MAX && logsThisTick != 0)
            {
                if (m_RegisteredMotors[m_IdToLog] != NULL)
                {
                    logsThisTick--;
                    double temp;
                    double encoderCt;
                    bool isInverted;
                    m_RegisteredMotors[m_IdToLog]->GetLogData(&temp, &encoderCt, &isInverted);

                    LogMotor(m_IdToLog, temp, encoderCt);
                }
                m_IdToLog++;
            }
            if (m_IdToLog >= REGISTERED_MOTORS_MAX)
            {
                m_IdToLog = 0;
            }
        }
    }

    /**
     * CowLogger::Reset
     * reset socket (is that necessary?) and refresh IP and port from constants
    */
    void CowLogger::Reset()
    {
        close(m_LogSocket);
        m_LogSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_LogSocket < 0)
        {
            std::cout << "CowLogger::CowLogger() error: failed to create socket" << std::endl;
            return;
        }

        ResetConstants();

        m_LogServer.sin_family      = AF_INET;
        m_LogServer.sin_addr.s_addr = m_LogServerIP;
        m_LogServer.sin_port        = m_LogServerPort;
    }

} /* namespace CowLib */
