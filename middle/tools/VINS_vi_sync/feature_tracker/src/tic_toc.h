// tic_toc.h
// A utility class for measuring elapsed time (in milliseconds)

#pragma once  // Prevent multiple inclusions of this header file

#include <ctime>          // For basic time-related functions
#include <cstdlib>        // For general utilities
#include <chrono>         // For high-resolution time measurement

// A class to measure the elapsed time between two points in code execution
class TicToc
{
  public:
    // Constructor: Automatically starts timing when an instance is created
    TicToc()
    {
        tic();  // Call tic() to initialize the start time
    }

    // Records the current time as the starting point of the timer
    void tic()
    {
        // Get the current system time and store it in 'start'
        start = std::chrono::system_clock::now();
    }

    // Calculates and returns the elapsed time (in milliseconds) since the last tic()
    double toc()
    {
        // Get the current system time as the end point
        end = std::chrono::system_clock::now();
        // Calculate the time difference between end and start
        std::chrono::duration<double> elapsed_seconds = end - start;
        // Convert the duration from seconds to milliseconds and return
        return elapsed_seconds.count() * 1000;
    }

  private:
    // Time points to store the start and end times of the measurement
    std::chrono::time_point<std::chrono::system_clock> start, end;
};