//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowTimer.h
// author: jon-bassi
// created on: 2022-1-15
// derived from wpi/Timer.h
//==================================================

#ifndef __COW_TIMER_H__
#define __COW_TIMER_H__

#include "frc/DriverStation.h"
#include "frc/RobotController.h"

#include <chrono>
#include <thread>

namespace CowLib
{

    /**
     * Pause the task for a specified time.
     *
     * Pause the execution of the program for a specified period of time given in
     * seconds. Motors will continue to run at their last assigned values, and
     * sensors will continue to update. Only the task containing the wait will pause
     * until the wait time is expired.
     *
     * @param seconds Length of time to pause, in seconds.
     */
    void CowWait(double seconds);

    /**
     * @brief  Gives real-time clock system time with nanosecond resolution
     * @return The time, in nanoseconds
     */
    double GetTime();

    /**
     * A timer class.
     *
     * Note that if the user calls frc::sim::RestartTiming(), they should also reset
     * the timer so Get() won't return a negative duration.
     */
    class CowTimer
    {
    public:
        /**
         * Create a new timer object.
         *
         * Create a new timer object and reset the time to zero. The timer is
         * initially not running and must be started.
         */
        CowTimer();

        virtual ~CowTimer() = default;

        CowTimer(const CowTimer &)            = default;
        CowTimer &operator=(const CowTimer &) = default;
        CowTimer(CowTimer &&)                 = default;
        CowTimer &operator=(CowTimer &&)      = default;

        /**
         * Get the current time from the timer. If the clock is running it is derived
         * from the current system clock the start time stored in the timer class. If
         * the clock is not running, then return the time when it was last stopped.
         *
         * @return Current time value for this timer in seconds
         */
        double Get() const;

        /**
         * Reset the timer by setting the time to 0.
         *
         * Make the timer startTime the current time so new requests will be relative
         * to now.
         */
        void Reset();

        /**
         * Start the timer running.
         *
         * Just set the running flag to true indicating that all time requests should
         * be relative to the system clock. Note that this method is a no-op if the
         * timer is already running.
         */
        void Start();

        /**
         * Stop the timer.
         *
         * This computes the time as of now and clears the running flag, causing all
         * subsequent time requests to be read from the accumulated time rather than
         * looking at the system clock.
         */
        void Stop();

        /**
         * Check if the period specified has passed.
         *
         * @param period The period to check.
         * @return       True if the period has passed.
         */
        bool HasElapsed(double) const;

        /**
         * Check if the period specified has passed and if it has, advance the start
         * time by that period. This is useful to decide if it's time to do periodic
         * work without drifting later by the time it took to get around to checking.
         *
         * @param period The period to check for.
         * @return       True if the period has passed.
         * @deprecated Use AdvanceIfElapsed() instead.
         */
        bool HasPeriodPassed(double period);

        /**
         * Check if the period specified has passed and if it has, advance the start
         * time by that period. This is useful to decide if it's time to do periodic
         * work without drifting later by the time it took to get around to checking.
         *
         * @param period The period to check for.
         * @return       True if the period has passed.
         */
        bool AdvanceIfElapsed(double period);

        /**
         * Return the FPGA system clock time in seconds.
         *
         * Return the time from the FPGA hardware clock in seconds since the FPGA
         * started. Rolls over after 71 minutes.
         *
         * @returns Robot running time in seconds.
         */
        static double GetFPGATimestamp();

        /**
         * Return the approximate match time.
         *
         * The FMS does not send an official match time to the robots, but does send
         * an approximate match time. The value will count down the time remaining in
         * the current period (auto or teleop).
         *
         * Warning: This is not an official time (so it cannot be used to dispute ref
         * calls or guarantee that a function will trigger before the match ends).
         *
         * The Practice Match function of the DS approximates the behavior seen on the
         * field.
         *
         * @return Time remaining in current match period (auto or teleop)
         */
        static double GetMatchTime();

    private:
        double m_StartTime       = 0;
        double m_AccumulatedTime = 0;
        bool m_Running           = false;
    };

} // namespace CowLib

#endif /* __COW_TIMER_H__ */