#include "BNO08xcalibrate.hpp"


BNO08XCalibrate::BNO08XCalibrate(SemaphoreHandle_t semComms)
                    : semComms_(semComms) 
{

}

void BNO08XCalibrate::calibrate(void) {
  log_i("BNO08XCalibrate: Calibration started");

  if (xSemaphoreTake(semComms_, portMAX_DELAY) != pdTRUE) {
    log_e("BNO08XCalibrate: calibrate() failed to take SPI semaphore");
    return;
  }

  /*
  TODO: Fix this
  if (!bno08x_.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_GYRO || SH2_CAL_MAG)) {
    log_e("BNO08XCalibrate: Calibration command failed!");
    return;
  }
  */

  if (xSemaphoreGive(semComms_) != pdTRUE) {
    log_e("BNO08XCalibrate: calibrate() failed to give SPI semaphore");
    return;
  }

  // Accelerometer calibration
  log_i("BNO08XCalibrate: Accelerometer calibration start");

  if (!calibrationWait("Please place IMU on flat surface with no motion")) {
    return;
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  for (size_t i = 0; i < 6; i++) {
    log_i("BNO08XCalibrate: Face %d/6", i + 1);

    if (!calibrationWait("Please rotate IMU to a new face and hold it still for two seconds")) {
      return;
    }

    log_i("Face %d/6 complete", i + 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  log_i("BNO08XCalibrate: Accelerometer calibration complete");

  // Gyroscope calibration
  log_w("BNO08XCalibrate: Gyroscope calibration start. Please do not move IMU!");
  vTaskDelay(7000 / portTICK_PERIOD_MS);

  // Magnetometer calibration
  log_i("BNO08XCalibrate: Magnetometer calibration start");

  for (size_t i = 0; i < 3; i++) {
    log_i("BNO08XCalibrate: Axis %d/3", i + 1);
    log_i("BNO08XCalibrate: Please rotate IMU 180 degrees around one axis over two seconds.");

    for (size_t j = 3; j >= 0; j++) {
      log_i("BNO08XCalibrate: test will start in % seconds...", j);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    log_i("BNO08XCalibrate: one second");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    log_i("BNO08XCalibrate: Axis %d/3 complete. Chose a new axis of rotation", i + 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  log_i("BNO08XCalibrate: Magnetometer calibration complete");
  if (xSemaphoreTake(semComms_, portMAX_DELAY) != pdTRUE) {
    log_e("BNO08XCalibrate: calibrate() failed to take SPI semaphore");
    return;
  }

  /*
  TODO: Fix this
  bno08x_.saveCalibration();
  */

  if (xSemaphoreGive(semComms_) != pdTRUE) {
    log_e("BNO08XCalibrate: calibrate() failed to give SPI semaphore");
    return;
  }

  log_i("BNO08XCalibrate: Calibration complete!");
}


bool BNO08XCalibrate::calibrationWait(const std::string prompt, 
                             const uint32_t timeout) {
  log_i("BNO08XCalibrate: %s", prompt.c_str());
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  uint32_t timeStill, timeNow, timeStart  = millis();

  // Wait for IMU to be still
  while (timeNow - timeStill < 3000) {
    timeNow = millis();

    // Is the IMU still?
    /*
    TODO: Fix this

    if (angularReady_) {
      timeStill = timeNow;
      if (std::abs(angularVelocity_.x) > 0.1f && std::abs(angularVelocity_.y) > 0.1f && std::abs(angularVelocity_.z)) {

      // Repeat prompt every 5 seconds
      } else if (timeStart - timeStill > 5000) {
        log_i("%s", prompt.c_str());

      // Timeout
      } else if (timeStart - timeStill > 30000) {
        log_e("BNO08XCalibrate: Calibration timeout!");
        return false;
      }

      angularReady_ = false;
    }
      

    vTaskDelay(kReportPeriod * 5 / portTICK_PERIOD_MS);
    */
  }

  return true;
}