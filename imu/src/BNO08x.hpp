#ifndef BNO08X_HPP
#define BNO08X_HPP

#include <Arduino.h>
#include "common/data.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <cstdint>
#include <array>
#include <string>
#include <memory>
#include <Wire.h>
#include <esp32-hal-log.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <ArduinoEigenDense.h>


/// @brief 
enum BNO08x_ResetReason {
  RESET_REASON_NONE     = 0,
  RESET_REASON_POR      = 1, // Power On Reset
  RESET_REASON_INTERNAL = 2, // Internal System Reset
  RESET_REASON_WATCHDOG = 3, // Watchdog Timeout
  RESET_REASON_EXTERNAL = 4, // External Reset Pin
  RESET_REASON_OTHER    = 5  // Other
};


class BNO08X {
public:
  /// @brief Makes a new I2C BNO08X object.
  /// @param pinReset Hardware reset pin.
  /// @param pinInt Interrupt pin.
  /// @param wire pointer to I2C bus.
  /// @param dataQueue IMU data queue handle.
  /// @param address I2C address.
  /// @param reportPeriod [ms] Time between IMU reports.
  BNO08X(const uint8_t pinReset,
         const uint8_t pinInt,
         std::shared_ptr<TwoWire> wire,
         QueueHandle_t dataQueue,
         const uint16_t reportPeriod = 10,
         const uint16_t address = 0x4B);
  
         
  /// @brief Makes a new I2C BNO08X object.
  /// @param pinReset Hardware reset pin.
  /// @param pinInt Interrupt pin.
  /// @param pinCS SPI chip select pin. Leave as 0 for I2C.
  /// @param spi pointer to SPI bus.
  /// @param dataQueue IMU data queue handle.
  /// @param reportPeriod [ms] Time between IMU reports.
  /// @param frequencySPI [Hz] SPI bus frequency. Defaults to BNO085 max, 3MHz.
  BNO08X(const uint8_t pinReset,
         const uint8_t pinInt,
         const uint8_t pinCS,
         std::shared_ptr<SPIClass> spi,
         QueueHandle_t dataQueue,
         const uint16_t reportPeriod = 10,
         const uint32_t frequencySPI = 3000000);

  
  /// @brief Task wrapper to call readReport. \n 
  ///        Used in ISR to resume receiveReport() task when report is ready.
  /// @param pvParameters 
  /// @return 
  static void IRAM_ATTR taskWrapperReadReport(void* pvParameters) {
    BNO08X* instance = static_cast<BNO08X*>(pvParameters);
    instance->readReport();
  };
  

private:
  const uint16_t kReportPeriod;
  const float kAccuracyQuat = 0.0610865f;       //!< [rad] From datasheet. Not used.
  const float kAccuracyLinear = 0.35f;          //!< [m/s^2] From datasheet.
  const float kAccuracyAngular = 0.0541052f;    //!< [rad/s] From datasheet.
  const uint8_t kPinInt_;                       //!< Interrupt pin.

  BNO08x bno08x_;
  bool quatReady_;
  bool angularReady_;
  bool linearReady_;
  Eigen::Quaternionf heading_;
  Eigen::Matrix3f covarianceHeading_;
  Eigen::Vector3f angularVelocity_;
  Eigen::Matrix3f covarianceAngular_;
  Eigen::Vector3f linearAcceleration_;
  Eigen::Matrix3f covarianceLinear_;
  TaskHandle_t taskReceiveReport_;
  TaskHandle_t taskResetCheck_;
  TaskHandle_t taskPostData_;
  SemaphoreHandle_t semComms_;
  SemaphoreHandle_t semData_;
  QueueHandle_t dataQueue_;
  

  /// @brief Setup IMU. Common constructor function for both I2C and SPI.
  /// @param dataQueue Queue to post IMU data to.
  void setup(QueueHandle_t dataQueue);


  /// @brief Sets report types.  \n 
  ///        WARNING: Does not take or release semaphore, must be done separately.
  void setReports(void);


  /// @brief Custom callback to handle sensor events from the SH2 driver. \n 
  ///        Needed to receive multiple reports at once.
  /// @param cookie Pointer to the BNO08X instance.
  /// @param pEvent Pointer to the sensor event data.
  static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);


  /// @brief Helper to convert Q-point fixed point to float.
  static float qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
    return fixedPointValue * std::pow(2.0f, -1.0f * qPoint);
  };


  /// @brief Checks if report is ready and updates data from same.
  /// @param pvParameters Not used.
  void receiveReport(void *pvParameters);


  /// @brief Task wrapper to call receiveReport.
  /// @param pvParameters Not used.
  static void taskWrapperReceiveReport(void* pvParameters) {
    BNO08X* instance = static_cast<BNO08X*>(pvParameters);
    instance->receiveReport(pvParameters); 
  };


  /// @brief Checks if IMU was reset and resets the reports.
  /// @param pvParameters Not used.
  void resetCheck(void *pvParameters);


  /// @brief Task wrapper to call resetCheck.
  /// @param pvParameters Not used.
  static void taskWrapperResetCheck(void* pvParameters) {
    BNO08X* instance = static_cast<BNO08X*>(pvParameters);
    instance->resetCheck(pvParameters); 
  };


  /// @brief 
  /// @param pvParameters std::pair<*bool, *ImuData> [data ready flag, data].
  void postData(void *pvParameters);


  /// @brief Task wrapper to call postData.
  /// @param pvParameters QueueHandle_t to post data to.
  static void taskWrapperPostData(void* pvParameters) {
    BNO08X* instance = static_cast<BNO08X*>(pvParameters);
    instance->postData(pvParameters); 
  };


  /// @brief Resumes the receive report task.
  void IRAM_ATTR readReport(void);
};

#endif // BNO08X_HPP
