#include "BNO08x.hpp"
#include "common/helpers.hpp"
#include <esp32-hal-log.h>
#include <cmath>

BNO08X::BNO08X(const uint8_t pinReset,
               const uint8_t pinInt,
               std::shared_ptr<TwoWire> wire,
               QueueHandle_t dataQueue,
               const uint16_t reportPeriod,
               const uint16_t address)
                : kReportPeriod(reportPeriod),
                  quatReady_(false),
                  angularReady_(false),
                  linearReady_(false),
                  kPinInt_(pinInt) {
  log_d("BNO08X: Constructor I2C");
  bool status = false;

  while (!status) {
    status = bno08x_.begin(address, *wire, pinInt, pinReset);

    if (!status) {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }

  log_d("BNO08X: Setup");
  setup(dataQueue);
}


BNO08X::BNO08X(const uint8_t pinReset,
               const uint8_t pinInt,
               const uint8_t pinCS,
               std::shared_ptr<SPIClass> spi,
               QueueHandle_t dataQueue,
               const uint16_t reportPeriod,
               const uint32_t frequencySPI)
                : kReportPeriod(reportPeriod),
                  quatReady_(false),
                  angularReady_(false),
                  linearReady_(false),
                  kPinInt_(pinInt) {
  log_d("BNO08X: Constructor SPI");
  bool status = false;

  while (!status) {
    log_d("BNO08X: Begin SPI");
    status = bno08x_.beginSPI(pinCS, pinInt, pinReset, frequencySPI, *spi);
    
    if (!status) {
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
  }

  log_d("BNO08X: Setup");
  setup(dataQueue);
}


void BNO08X::setup(QueueHandle_t dataQueue) {
  this->dataQueue_ = dataQueue;
  covarianceHeading_ = Helpers::calculateCovariance(kAccuracyQuat);
  covarianceAngular_ = Helpers::calculateCovariance(kAccuracyAngular);
  covarianceLinear_ = Helpers::calculateCovariance(kAccuracyLinear);

  #ifdef __PLATFORMIO_BUILD_DEBUG__
    log_d("BNO08X: Debug mode enabled");
    bno08x_.enableDebugging();  // Enable debug mode for IMU
    vTaskDelay(100 / portTICK_PERIOD_MS);
  #endif

  log_d("Creating semaphores...");
  semComms_ = xSemaphoreCreateMutex();
  semData_ = xSemaphoreCreateMutex();

  if (semComms_ == nullptr || semData_ == nullptr) {
    log_e("BNO08X: Failed to create semaphore");
    while (1);
  }

  log_d("Semaphores created");
  log_d("Creating IMU tasks...");
  Helpers::takeSemaphore(semComms_, "BNO08X comms", "setup");

  // Start tasks
  BaseType_t success = pdFAIL;
  success = xTaskCreate(BNO08X::taskWrapperReceiveReport,
                        "receiveReport",
                        10 * 1024,
                        this,
                        priorityLevels::PRIORITY_HIGHEST,
                        &taskReceiveReport_);
                      
  if (success != pdTRUE ) {
    log_e("BNO08X: Failed to create receiveReport task");
    while (1);
  }

  log_d("Receive report task created");
  success = xTaskCreate(BNO08X::taskWrapperPostData,
                        "postData",
                        10 * 1024,
                        this,
                        priorityLevels::PRIORITY_HIGH,
                        &taskPostData_);
  
  if (success != pdTRUE ) {
    log_e("BNO08X: Failed to create postData task");
    while (1);
  }

  log_d("Post data task created");
  success = xTaskCreate(BNO08X::taskWrapperResetCheck,
                        "resetCheck",
                        10 * 1024,
                        this,
                        priorityLevels::PRIORITY_HIGH,
                        &taskResetCheck_);

  if (success != pdTRUE ) {
    log_e("BNO08X: Failed to create resetCheck task");
    while (1);
  }

  log_d("Reset check task created");
  log_d("IMU tasks Created");
  log_d("BNO08X: Servicing bus");
  bno08x_.serviceBus();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  
  // Register custom callback to handle batched reports correctly
  sh2_setSensorCallback(sensorHandler, this);

  // Clear initial flags
  log_d("BNO08X: Clear tare");
  bno08x_.clearTare();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  log_d("BNO08X: Clear reset flag");
  bno08x_.wasReset();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  log_d("BNO08X: Clear event flag");

  if(bno08x_.getSensorEvent()) {
    bno08x_.getSensorEventID();
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  Helpers::giveSemaphore(semComms_, "BNO08X comms", "setup");
  log_d("BNO08X: Enabling reports");
  setReports();
  log_d("BNO08X: Starting tasks");
  vTaskResume(taskPostData_);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  vTaskResume(taskReceiveReport_);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  vTaskResume(taskResetCheck_);
  log_d("BNO08X: Tasks started");
  log_d("BNO08X: Setup complete");
}


void BNO08X::setReports(void) {
  if (!bno08x_.enableLinearAccelerometer(kReportPeriod)) {
    log_e("BNO08X: Failed to enable Linear Accel");
  }

  vTaskDelay(50 / portTICK_PERIOD_MS);

  if (!bno08x_.enableGyro(kReportPeriod)) {
    log_e("BNO08X: Failed to enable Gyro");
  }

  vTaskDelay(50 / portTICK_PERIOD_MS);

  if (!bno08x_.enableRotationVector(kReportPeriod)) {
    log_e("BNO08X: Failed to enable Rotation Vector");
  }
}


void BNO08X::receiveReport(void *pvParameters) {
  vTaskSuspend(NULL); // Wait for tasks to be started.
  log_d("BNO08X: Receive report task started");
  const std::string taskName = pcTaskGetName(NULL);

  for(;;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait until triggered by ISR.
    Helpers::takeSemaphore(semComms_, "BNO08X comms", taskName);
    
    while (digitalRead(kPinInt_) == LOW) { // Low indicates new message ready.
      bno08x_.serviceBus(); // This triggers sensorHandler for all reports in the packet
    }

    Helpers::giveSemaphore(semComms_, "BNO08X comms", taskName);
    Helpers::takeSemaphore(semData_, "BNO08X data", taskName);

    if (linearReady_ && angularReady_ && quatReady_) {
      // Post data
      xTaskNotifyGive(taskPostData_);
      linearReady_ = angularReady_ = quatReady_ = false;
    }

    Helpers::giveSemaphore(semData_, "BNO08X data", taskName);
  }
}


void BNO08X::resetCheck(void *pvParameters) {
  vTaskSuspend(NULL); // Wait for tasks to be started.
  log_d("BNO08X: Reset check task started");
  const std::string taskName = pcTaskGetName(NULL);
  const TickType_t taskDelayPeriod = 500;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, taskDelayPeriod);
    lastWakeTime = xTaskGetTickCount();
    log_v("BNO08X: Checking for IMU reset");
    Helpers::takeSemaphore(semComms_, "BNO08X comms", taskName);

    if (bno08x_.wasReset()) {
      BNO08x_ResetReason errrorReason = static_cast<BNO08x_ResetReason>(bno08x_.getResetReason());
      std::string errorString = "BNO08X: IMU was reset: ";
      switch (errrorReason) {
        case RESET_REASON_NONE:
          log_w("%s %s", errorString.c_str(), "No reason given");
          break;

        case RESET_REASON_POR:
          log_w("%s %s", errorString.c_str(), "Power On Reset");
          break;

        case RESET_REASON_INTERNAL:
          log_w("%s %s", errorString.c_str(), "Internal System Reset");
          break;
        
        case RESET_REASON_WATCHDOG:
          log_w("%s %s", errorString.c_str(), "Watchdog Timeout");
          break;

        case RESET_REASON_EXTERNAL:
          log_w("%s %s", errorString.c_str(), "External Reset Pin");
          break;

        case RESET_REASON_OTHER:
          log_w("%s %s", errorString.c_str(), "Other");
          break;

        default:
          log_e("%s %s", errorString.c_str(), "Unknown reason");
          break;
      }

      setReports();
    }

    Helpers::giveSemaphore(semComms_, "BNO08X comms", taskName);
    log_v("BNO08X: IMU reset check complete");
  }
}


void BNO08X::postData(void *pvParameters) {
  vTaskSuspend(NULL); // Wait for tasks to be started.
  const std::string taskName = pcTaskGetName(NULL);
  const TickType_t queueWaitPeriod = 20;  // [ms]
  
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Only runs when triggered by data ready

    if (uxQueueSpacesAvailable(dataQueue_) <= 0) {
      log_e("BNO08X: IMU data queue is full");

    } else {
      log_v("BNO08X: Posting data to queue");
      Helpers::takeSemaphore(semData_, "BNO08X data", taskName);
      ImuData data(heading_, 
                  covarianceHeading_, 
                  angularVelocity_, 
                  covarianceAngular_, 
                  linearAcceleration_,
                  covarianceLinear_);

      if (xQueueSend(dataQueue_, static_cast<void*>(&data), queueWaitPeriod) != pdTRUE ) {
        log_e("BNO08X: IMU data failed to post to queue");
      }
    }

    linearReady_ = false;
    angularReady_ = false;
    quatReady_ = false;
    Helpers::giveSemaphore(semData_, "BNO08X data", taskName);
  }
}


void IRAM_ATTR BNO08X::readReport(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(taskReceiveReport_, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}


void BNO08X::sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent) {
  BNO08X *self = static_cast<BNO08X*>(cookie);
  Helpers::takeSemaphore(self->semData_, "BNO08X data", "sensorHandler");

  // Helper to read values from the byte array (Little Endian).
  auto read16 = [](const uint8_t *data, uint8_t index) -> int16_t {
    uint16_t combined = static_cast<uint16_t>(data[index]) | 
                        (static_cast<uint16_t>(data[index + 1]) << 8);
    return (int16_t)combined;
  };

  switch (pEvent->reportId) {
    case SH2_ROTATION_VECTOR: {
      Eigen::Quaternionf quat;
      quat.w() = qToFloat(read16(pEvent->report, 10), 14);
      quat.x() = qToFloat(read16(pEvent->report, 4), 14);
      quat.y() = qToFloat(read16(pEvent->report, 6), 14);
      quat.z() = qToFloat(read16(pEvent->report, 8), 14);
      self->heading_ = quat;
      self->quatReady_ = true;
      break;
    }

    case SH2_GYROSCOPE_CALIBRATED: {
      Eigen::Vector3f vector;
      vector.x() = qToFloat(read16(pEvent->report, 4), 9);
      vector.y() = qToFloat(read16(pEvent->report, 6), 9);
      vector.z() = qToFloat(read16(pEvent->report, 8), 9);
      self->angularVelocity_ = vector;
      self->angularReady_ = true;
      break;
    }

    case SH2_LINEAR_ACCELERATION: {
      Eigen::Vector3f vector;
      vector.x() = qToFloat(read16(pEvent->report, 4), 8);
      vector.y() = qToFloat(read16(pEvent->report, 6), 8);
      vector.z() = qToFloat(read16(pEvent->report, 8), 8);
      self->linearAcceleration_ = vector;
      self->linearReady_ = true;
      break;
    }

    default:
      log_w("BNO08X: Received unhandled report ID: %d", pEvent->reportId);
      break;
  }

  Helpers::giveSemaphore(self->semData_, "BNO08X data", "sensorHandler");
}