/*
 * CowGyro.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: kchau
 */

#include "CowGyro.h"

#include "CowLib/CowLogger.h"
#include "CowLib/CowTimer.h"

#include <algorithm>
#include <cmath>
#include <frc/RobotController.h>
#include <frc/SPI.h>
#include <iostream>
#include <vector>

namespace CowLib
{

    std::vector<e_ErrorFlag> CowGyro::m_ALL_ERRORS
        = { PLL_FAILURE, QUADRATURE_ERROR, NONVOLATILE_MEMORY_FAULT, RESET_INITIALIZE_FAILURE, POWER_FAILURE, CONTINUOUS_SELFTEST_FAILURE, GENERATED_FAULTS };

    int32_t CowGyro::m_RemainingStartupCycles = CowGyro::K_STARTUP_SAMPLES;

    double CowGyro::m_Angle        = 0;
    double CowGyro::m_LastTime     = 0;
    double CowGyro::m_VolatileRate = 0;
    double CowGyro::m_ZeroBias     = 0;
    double CowGyro::m_ZeroRatesSamples[K_ZEROING_SAMPLES];

    bool CowGyro::m_IsZeroed                = false;
    bool CowGyro::m_Calibrating             = false;
    bool CowGyro::m_HasEnoughZeroingSamples = false;

    uint16_t CowGyro::m_CurrentIndex = 0;

    CowGyro *CowGyro::m_Instance = NULL;
    frc::SPI *CowGyro::m_Spi     = NULL;

    CowGyro::CowGyro()
    {
        CowLib::CowWait(0.25);

        m_Spi = new frc::SPI(frc::SPI::kMXP);
        m_Spi->SetClockRate(4000000);
        m_Spi->SetChipSelectActiveLow();

        // deprecated
        // This is sometimes called clock polarity low or clock idle low.
        // m_Spi->SetClockActiveHigh();
        // replace with following
        // Mode 0 is Clock idle low, data sampled on rising edge
        m_Spi->SetMode(frc::SPI::kMode0);

        // deprecated, does not work
        // m_Spi->SetMSBFirst();
        CowLib::CowWait(2);

        m_Thread = new std::thread(CowGyro::Handle);
    }

    CowGyro::~CowGyro()
    {
        m_Thread->detach();
        delete m_Thread;
    }

    void CowGyro::Handle()
    {
        bool initialized = false;

        while (!initialized)
        {
            initialized = InitializeGyro();
            if (!initialized)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }

        std::cout << "Gyro is initialized! PID: 0x" << std::hex << ReadPartId() << std::endl;

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds((int) ((1.0 / K_READING_RATE) * 1000)));
            int reading = GetReading();

            e_StatusFlag status = CowGyro::ExtractStatus(reading);

            std::vector<e_ErrorFlag> errors = CowGyro::ExtractErrors(reading);

            if ((e_StatusFlag::VALID_DATA != status) || !errors.empty())
            {
                std::cerr << "Results: " << reading << ". ";
                std::cerr << "Gyro read failed. Status: " << status << ". Errors: " << errors.size();
                for (uint32_t i = 0; i < errors.size(); i++)
                {
                    std::cerr << " , " << i << ": " << errors[i];
                }
                std::cerr << std::endl;
                continue;
            }

            if (m_RemainingStartupCycles > 0)
            {
                m_RemainingStartupCycles--;
                continue;
            }

            double unbiasedAngleRate = CowGyro::ExtractAngleRate(reading);

            // Add zeroing samples if calibrating gyro
            if (m_Calibrating)
            {
                m_ZeroRatesSamples[m_CurrentIndex] = unbiasedAngleRate;
                m_CurrentIndex++;
                if (m_CurrentIndex >= K_ZEROING_SAMPLES)
                {
                    m_CurrentIndex            = 0;
                    m_HasEnoughZeroingSamples = true;
                }
            }

            // Calculate
            if (m_IsZeroed)
            {
                if (m_LastTime == 0)
                {
                    m_LastTime = CowLib::CowTimer::GetFPGATimestamp();
                }

                double currentTime = CowLib::CowTimer::GetFPGATimestamp();
                double timeElapsed = currentTime - m_LastTime;
                m_LastTime         = currentTime;

                m_VolatileRate = unbiasedAngleRate - m_ZeroBias;

                // Don't integrate anything less than 80 LSB / degree / sec
                if (fabs(m_VolatileRate) < 0.0125)
                {
                    m_VolatileRate = 0;
                }
                m_Angle += m_VolatileRate * timeElapsed;
            }
        }
    }

    float CowGyro::GetAngle()
    {
        return m_Angle;
    }

    double CowGyro::GetRate()
    {
        return m_VolatileRate;
    }

    bool CowGyro::InitializeGyro()
    {
        // Start a self-check
        DoTransaction(SENSOR_DATA_CMD | CHK_GENERATE_FAULTS_BIT);
        //    if(result != 1)
        //    {
        //        std::cerr << "Unexpected self check response " << std::hex << result << std::endl;
        //        return false;
        //    }

        // Wait for the fault conditions to occur
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Clear latched non-fault data
        DoTransaction(SENSOR_DATA_CMD);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Actually read the self-test data
        int32_t selfCheckResult = DoTransaction(SENSOR_DATA_CMD);
        if (ExtractStatus(selfCheckResult) != SELF_TEST_DATA)
        {
            // std::cerr << "Gyro not in self test: 0x" << std::hex << selfCheckResult <<std::endl;
            return false;
        }

        std::vector<e_ErrorFlag> errors = ExtractErrors(selfCheckResult);
        bool containsAllErrors          = std::includes(m_ALL_ERRORS.begin(), m_ALL_ERRORS.end(), errors.begin(), errors.end());
        if (!containsAllErrors)
        {
            // std::cerr << "Gyro self-test didn't include all errors: 0x" << std::hex << selfCheckResult << std::endl;
            return false;
        }

        // Clear the latched self-test data
        selfCheckResult = DoTransaction(SENSOR_DATA_CMD);
        if (ExtractStatus(selfCheckResult) != SELF_TEST_DATA)
        {
            std::cerr << "Gyro second self test read failed: 0x" << std::hex << selfCheckResult << std::endl;
            return false;
        }

        return true;
    }

    int16_t CowGyro::DoRead(int8_t address)
    {
        int32_t command = (0x8 << 28) | (address << 17);
        while (true)
        {
            int32_t result;
            result = DoTransaction(command);
            if ((result & 0xEFE00000) != 0x4E000000)
            {
                std::cerr << "Unexpected gyro read response: 0x" << std::hex << result << std::endl;
                continue;
            }
            return (int16_t) ((result >> 5) & 0xFFFF);
        }
    }

    double CowGyro::ExtractAngleRate(int32_t result)
    {
        int16_t reading = (int16_t) ((result >> 10) & 0xFFFF);
        return ((double) (reading) / 80.0);
    }

    int16_t CowGyro::ReadPartId()
    {
        return DoRead((int8_t) 0x0C);
    }

    int32_t CowGyro::GetReading()
    {
        return DoTransaction(SENSOR_DATA_CMD);
    }

    int32_t CowGyro::DoTransaction(int32_t command)
    {
        if (!IsOddParity(command & ~0x01))
        {
            command |= 0x01;
        }
        uint8_t commandArray[4];

        // Convert command into byte array
        commandArray[0] = (command >> 24) & 0xFF;
        commandArray[1] = (command >> 16) & 0xFF;
        commandArray[2] = (command >> 8) & 0xFF;
        commandArray[3] = command & 0xFF;

        uint8_t resultBuffer[4];

        int transactionSize = CowGyro::m_Spi->Transaction(commandArray, resultBuffer, 4);

        if (transactionSize != 4)
        {
            std::cerr << "Transaction failed with size: " << transactionSize << std::endl;
        }

        int32_t result = 0;

        // Convert resultBuffer into int
        result = (int32_t) resultBuffer[3];
        result |= (resultBuffer[0] << 24);
        result |= (resultBuffer[1] << 16);
        result |= (resultBuffer[2] << 8);

        if (!IsOddParity(result))
        {
            // std::cerr << "High bytes parity failures" << std::endl;
        }

        if (!IsOddParity(result))
        {
            // std::cerr << "Whole word parity failure" << std::endl;
        }

        return result;
    }

    bool CowGyro::IsOddParity(int32_t word)
    {
        bool isOdd = false;
        for (int i = 0; i < 32; ++i)
        {
            if ((word & (1 << i)) != 0)
            {
                isOdd = !isOdd;
            }
        }
        return isOdd;
    }

    e_StatusFlag CowGyro::ExtractStatus(int32_t result)
    {
        int stBits = (result >> 26) & 0b11;
        switch (stBits)
        {
        case 0b00 :
            return INVALID_DATA;
        case 0b01 :
            return VALID_DATA;
        case 0b10 :
            return SELF_TEST_DATA;
        case 0b11 :
            return RW_RESPONSE;
        default :
            std::cerr << "whiskey tango foxtrot" << std::endl;
        }
        return STATUS_FLAG_ERROR;
    }

    std::vector<e_ErrorFlag> CowGyro::ExtractErrors(int result)
    {
        std::vector<e_ErrorFlag> errors;
        std::vector<e_ErrorFlag>::iterator it;

        for (int i = PLL_FAILURE; i <= GENERATED_FAULTS; ++i)
        {
            if ((result & (1 << i)) != 0)
            {
                errors.insert(errors.begin(), (e_ErrorFlag) i);
            }
        }

        std::sort(errors.begin(), errors.end());
        return errors;
    }

    void CowGyro::Reset()
    {
        m_Angle        = 0;
        m_CurrentIndex = 0;
        m_VolatileRate = 0;
        m_ZeroBias     = 0;
    }

    void CowGyro::ResetAngle()
    {
        m_Angle        = 0;
        m_VolatileRate = 0;
    }

    void CowGyro::BeginCalibration()
    {
        m_Calibrating = true;
        m_IsZeroed    = false;
        Reset();
    }

    void CowGyro::FinalizeCalibration()
    {
        m_Calibrating = false;
        m_IsZeroed    = true;

        int32_t index = m_CurrentIndex;

        if (m_HasEnoughZeroingSamples)
        {
            index                     = K_ZEROING_SAMPLES;
            m_HasEnoughZeroingSamples = false;
            std::cout << "Have enough zeroing samples!" << std::endl;
        }
        else
        {
            std::cout << "DO NOT have enough zeoring samples, got " << std::dec << index << std::endl;
        }

        // Average the samples in circular buffer
        for (int i = 0; i < index; ++i)
        {
            m_ZeroBias += (m_ZeroRatesSamples[i] / K_ZEROING_SAMPLES);
            m_ZeroRatesSamples[i] = 0;
        }

        // std::cout << "attempting to log gyro" << std::endl;
        // CowLogger::GetInstance()->Log("Gyro: Zero Bias", m_ZeroBias);
        // std::cout << "finished to log gyro" << std::endl;
        m_LastTime = CowLib::CowTimer::GetFPGATimestamp();

        // std::cout << "Finalized gyro, angle: " << m_Angle << " bias: " << m_ZeroBias << std::endl;
    }

    CowGyro *CowGyro::GetInstance()
    {
        if (m_Instance == 0)
        {
            m_Instance = new CowGyro();
        }

        return m_Instance;
    }

} /* namespace CowLib */
