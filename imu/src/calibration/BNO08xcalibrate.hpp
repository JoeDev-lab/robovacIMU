#ifndef BNO08XCALIBRATE_HPP
#define BNO08XCALIBRATE_HPP

#include <Arduino.h>
#include "../common/data.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <cstdint>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <ESP32Time.h>


// TODO: Finish this class


class BNO08XCalibrate {
public:
  BNO08XCalibrate(SemaphoreHandle_t semComms);

  /// @brief Runs the IMU though the calibration process. \n 
  ///        This is a blocking function. \n
  ///        1. Place IMU on flat surface with no motion. \n
  ///        2. [Accelerometer] Call calibration function.  \n
  ///         2a. Wait for prompt (approx 1 second).  \n 
  ///         2b. Rotate IMU so that all 6 'faces' of the IMU have been covered and repeat steps 2-4. \n
  ///        3. [Gyroscope] Wait for prompt (approx 7 seconds). \n
  ///        4. [Magnetometer] rotate IMU 180 degrees (take approx 2 seconds to rotate). \n
  ///         4a. Wait for prompt (approx 1 second).  \n
  ///         4b. Repeat steps 6-7 for the remaining two axes.
  void calibrate(void);

private:
  SemaphoreHandle_t semComms_ = nullptr;

  /// @brief Waits for accelerometer calibration to be held still. Will timeout after set duration.
  /// @param prompt Prompt for the user.
  /// @param timeout [ms] Timeout period.
  /// @return False on timeout.
  bool calibrationWait(const std::string prompt, 
                       const uint32_t timeout = 30000);
};

#endif // BNO08XCALIBRATE_HPP
