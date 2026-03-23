#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp32-hal-log.h>
#include <stdint.h>
#include <string>
#include <ArduinoEigenDense.h>


namespace Helpers {
  /// @brief Take a semaphore and log an error if it fails.
  /// @param semaphoreHandle Handle to the semaphore.
  /// @param semaphoreName The name of the semaphore. Used in error log.
  /// @param taskName The name of the calling task. Used in error log.
  /// @param callingFunction The name of the calling function. Used in error log.
  static void takeSemaphore(SemaphoreHandle_t semaphoreHandle, 
                            std::string semaphoreName, 
                            std::string taskName) 
  {
    if (xSemaphoreTake(semaphoreHandle, portMAX_DELAY) != pdTRUE) {
      log_e("%s: Failed to take %s semaphore", 
            taskName.c_str(), 
            semaphoreName.c_str());
      log_e("Held by task: %s", pcTaskGetName(xSemaphoreGetMutexHolder(semaphoreHandle)));
    }
  }


  /// @brief give a semaphore and retry if it fails.
  /// @param semaphoreHandle Handle to the semaphore.
  /// @param semaphoreName The name of the semaphore. Used in error log.
  /// @param taskName The name of the calling task. Used in error log.
  /// @param callingFunction The name of the calling function. Used in error log.
  static void giveSemaphore(SemaphoreHandle_t semaphoreHandle, 
                            std::string semaphoreName, 
                            std::string taskName) 
  {
    while (xSemaphoreGive(semaphoreHandle) != pdTRUE) {
      log_e("%s: Failed to give %s semaphore. Retrying...", 
            taskName.c_str(), 
            semaphoreName.c_str());
      vTaskDelay(10);
    }
  }


  /// @brief Extract the roll component from a quaternion.
  /// @param quat quaternion.
  /// @return roll component.
  static float quatToRoll(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t0 = +2.0f * ((normalisedQuat.w() * normalisedQuat.x()) + (normalisedQuat.y() * normalisedQuat.z()));
    float t1 = +1.0f - 2.0f * (normalisedQuat.x() * normalisedQuat.y() + squaredY);
    float roll = atan2(t0, t1);

    return roll;
  }


  /// @brief Extract the pitch component from a quaternion.
  /// @param quat quaternion.
  /// @return pitch component.
  static float quatToPitch(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t2 = 2.0f * ((normalisedQuat.w() * normalisedQuat.y()) - (normalisedQuat.z() * normalisedQuat.x()));
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2);

    return pitch;
  }


  /// @brief Extract the yaw component from a quaternion.
  /// @param quat quaternion.
  /// @return yaw component.
  static float quatToYaw(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t3 = 2.0f * ((normalisedQuat.w() * normalisedQuat.z()) + (normalisedQuat.x() * normalisedQuat.y()));
    float t4 = 1.0f - 2.0f * (squaredY + (normalisedQuat.z() * normalisedQuat.z()));
    float yaw = atan2(t3, t4);
    
    return yaw;
  }


  /// @brief Convert a quaternion to roll, pitch and yaw.
  /// @param quat quaternion.
  /// @return [roll, pitch, yaw]
  static std::array<float, 3> quatToEuler(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    // roll (x-axis rotation)
    float t0 = 2.0f * ((normalisedQuat.w() * normalisedQuat.x()) + (normalisedQuat.y() * normalisedQuat.z()));
    float t1 = 1.0f - 2.0f * ((normalisedQuat.x() * normalisedQuat.y()) + squaredY);
    float roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = 2.0f * ((normalisedQuat.w() * normalisedQuat.y()) - (normalisedQuat.z() * normalisedQuat.x()));
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2);

    // yaw (z-axis rotation)
    float t3 = 2.0f * ((normalisedQuat.w() * normalisedQuat.z()) + (normalisedQuat.x() * normalisedQuat.y()));
    float t4 = 1.0f - 2.0f * (squaredY + (normalisedQuat.z() * normalisedQuat.z()));
    float yaw = atan2(t3, t4);

    return {roll, pitch, yaw};
  }


  /// @brief Generates covariance matrix heuristic from acuracy of sensor and reported acuracy level.
  /// @param sensorAcuracy The acuracy of the sensor in the datasheet.
  /// @return Eigen::Matrix3f Covariance matrix.
  static Eigen::Matrix3f calculateCovariance(const float sensorAcuracy) {
    Eigen::Matrix3f covariance;
    float variance = sensorAcuracy * sensorAcuracy;
    covariance(0, 0) = variance;  // X
    covariance(1, 1) = variance;  // Y
    covariance(2, 2) = variance;  // Z

    return covariance;
  }
}

#endif // HELPERS_HPP
