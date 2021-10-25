//////////////////////////////////////////////////////////////////////////////////////////
/// Provides the header of the cycle timer class. A simple cycle time handler 
/// 
/// \file cycleTimer.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <chrono>

namespace RIB
{
    class CycleTimer
    {
    private:
        std::chrono::nanoseconds m_cycleTime;
        std::chrono::nanoseconds m_lastCycleTime = std::chrono::nanoseconds::zero();
        std::chrono::nanoseconds m_minCycleTime = std::chrono::nanoseconds::max();
        std::chrono::nanoseconds m_maxCycleTime = std::chrono::nanoseconds::min();
        std::chrono::nanoseconds m_avgCycleTime = std::chrono::nanoseconds::zero();
        std::chrono::nanoseconds m_lastLoopTime = std::chrono::nanoseconds::zero();
        std::chrono::nanoseconds m_minLoopTime = std::chrono::nanoseconds::max();
        std::chrono::nanoseconds m_maxLoopTime = std::chrono::nanoseconds::min();
        std::chrono::nanoseconds m_avgLoopTime = std::chrono::nanoseconds::zero();
        std::chrono::nanoseconds m_cycleTimeDeviation;
        std::chrono::time_point<std::chrono::steady_clock>  m_cycleStartTime = std::chrono::steady_clock::now();

        ///
        /// Calculates the min, max and avg loop time
        ///
        void UpdateLoopTimeValues();

        ///
        /// Calculates the min, max and avg cycle time
        ///
        void UpdateCycleTimeValues();

    protected:
        ///
        /// Calculates the cycle time and restarts the cycle
        /// The cycle time is the time from Begin() to end of cycle
        ///
        void RestartCycle();

        ///
        /// Calculate the loop time 
        /// the loop time is the time from Begin() to WaitForElapse()
        ///
        void GetLoopTime();

        virtual std::chrono::steady_clock::time_point GetCurrentTime();

        virtual void SleepUntil(std::chrono::steady_clock::time_point timePoint);


    public:
        ///
        /// Constructor
        ///
        CycleTimer();

        ///
        /// Constructor
        /// \param cycleTime     Value for cycle time in microseconds
        ///
        CycleTimer(const std::chrono::microseconds cycleTime);

        ///
        /// Constructor
        /// \param cycleTime     Value for cycle time in microseconds
        /// \param cycleTimeDeviation     Value for cycle time deviation in microseconds
        /// \throw std::invalid_argument when cycleTime is negative
        ///
        CycleTimer(const std::chrono::microseconds cycleTime, const std::chrono::microseconds cycleTimeDeviation);

        virtual ~CycleTimer() = default;

        ///
        /// SetCycletime
        /// \param cycletime     Value for cycletime in microseconds
        /// \throw std::invalid_argument when cycleTime is negative
        ///
        void SetCycleTime(const std::chrono::microseconds cycleTime);

        ///
        /// SetCycletimeDeviation
        /// \param cycletimedeviation Value for maximum deviation of cycletime in microseconds
        /// \throw std::invalid_argument when cycleTimeDeviation is negative
        ///
        void SetCycleTimeDeviation(const std::chrono::microseconds cycleTimeDeviation);

        ///
        /// Startpoint for cycle time control. It takes the current time point and 
        /// should be called directly upfront the loop
        ///
        void Begin();

        ///
        /// Waits until the specified cycle time is over
        /// Should be the last line in the loop
        /// /return     if false cycle time exceeded cycle time + deviation
        ///
        bool WaitForElapse();

        ///
        /// Getter for the cycle time 
        ///
        template < typename T >
        T GetCycleTime() const
        {
            return std::chrono::duration_cast<T>(m_cycleTime);
        }

        ///
        /// Getter for the last cycle time 
        ///
        template < typename T >
        T GetLastCycleTime() const
        {
            return std::chrono::duration_cast<T>(m_lastCycleTime);
        }

        ///
        /// Getter for the minimum cycle time 
        /// Should be always equal to the cycle time
        ///
        template < typename T >
        T GetMinCycleTime() const
        {
            return std::chrono::duration_cast<T>(m_minCycleTime);
        }

        ///
        /// Getter for the maximum cycle time 
        /// 
        template < typename T >
        T GetMaxCycleTime() const
        {
            return std::chrono::duration_cast<T>(m_maxCycleTime);
        }

        ///
        /// Getter for the average cycle time 
        ///
        template < typename T >
        T GetAvgCycleTime() const
        {
            return std::chrono::duration_cast<T>(m_avgCycleTime);
        }

        ///
        /// Getter for the last loop time 
        ///
        template < typename T >
        T GetLastLoopTime() const
        {
            return std::chrono::duration_cast<T>(m_lastLoopTime);
        }

        ///
        /// Getter for the minimum loop time 
        ///
        template < typename T >
        T GetMinLoopTime() const
        {
            return std::chrono::duration_cast<T>(m_minLoopTime);
        }

        ///
        /// Getter for the maximum loop time 
        ///
        template < typename T >
        T GetMaxLoopTime() const
        {
            return std::chrono::duration_cast<T>(m_maxLoopTime);
        }

        ///
        /// Getter for the average loop time 
        ///
        template < typename T >
        T GetAvgLoopTime() const
        {
            return std::chrono::duration_cast<T>(m_avgLoopTime);
        }

        ///
        /// Getter for the cycle deviation time 
        ///
        template < typename T >
        T GetCycleTimeDeviation() const
        {
            return std::chrono::duration_cast<T>(m_cycleTimeDeviation);
        }

        std::chrono::time_point<std::chrono::steady_clock> GetCycleStartTime() const;
    };

}
