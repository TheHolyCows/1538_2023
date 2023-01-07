/*
 * CowGyro.h
 *
 *  Created on: Jan 22, 2016
 *      Author: kchau
 */

#ifndef SRC_COWGYRO_H_
#define SRC_COWGYRO_H_

#include <frc/SPI.h>
#include <thread>
#include <vector>

namespace CowLib
{

    typedef enum
    {
        INVALID_DATA,
        VALID_DATA,
        SELF_TEST_DATA,
        RW_RESPONSE,
        STATUS_FLAG_ERROR
    } e_StatusFlag;

    typedef enum
    {
        PLL_FAILURE                 = 7,
        QUADRATURE_ERROR            = 6,
        NONVOLATILE_MEMORY_FAULT    = 5,
        RESET_INITIALIZE_FAILURE    = 4,
        POWER_FAILURE               = 3,
        CONTINUOUS_SELFTEST_FAILURE = 2,
        GENERATED_FAULTS            = 1
    } e_ErrorFlag;

    class CowGyro
    {
    private:
        std::thread *m_Thread;
        static CowGyro *m_Instance;
        static frc::SPI *m_Spi;

        static std::vector<e_ErrorFlag> m_ALL_ERRORS;

        static const int SENSOR_DATA_CMD         = 0x20000000;
        static const int CHK_GENERATE_FAULTS_BIT = 0x03;

        static const int32_t K_READING_RATE    = 1000;
        static const int32_t K_ZEROING_SAMPLES = 5 * K_READING_RATE;
        static const int32_t K_STARTUP_SAMPLES = 2 * K_READING_RATE;

        static int32_t m_RemainingStartupCycles;

        static double m_Angle;
        static double m_LastTime;
        static double m_VolatileRate;
        static double m_ZeroBias;
        static double m_ZeroRatesSamples[K_ZEROING_SAMPLES];

        static bool m_Calibrating;
        static bool m_HasEnoughZeroingSamples;
        static bool m_IsZeroed;

        static uint16_t m_CurrentIndex;

    public:
        CowGyro();
        virtual ~CowGyro();
        float GetAngle();
        double GetRate();
        void ResetAngle();
        static void BeginCalibration();
        static void FinalizeCalibration();
        static CowGyro *GetInstance();

    private:
        static void Handle();
        static int16_t DoRead(int8_t address);
        static int32_t DoTransaction(int32_t command);
        static std::vector<e_ErrorFlag> ExtractErrors(int32_t result);
        static e_StatusFlag ExtractStatus(int32_t result);
        static double ExtractAngleRate(int32_t result);
        static int32_t GetReading();
        static bool InitializeGyro();
        static bool IsOddParity(int32_t word);
        static int16_t ReadPartId();
        static void Reset();
    };

} /* namespace CowLib */

#endif /* SRC_COWGYRO_H_ */
