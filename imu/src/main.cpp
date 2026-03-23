// Communication method
#define IMU_COMS 1  // 0 = I2C, 1 = SPI


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Arduino.h>
#include <cstdint>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include "common/data.hpp"
#include "common/helpers.hpp"
#include "BNO08x.hpp"
#include "ENC28J60.hpp"
#include <esp32-hal-log.h>
#include <memory>
#include <SPI.h>
#include <Wire.h>
#include <EthernetESP32.h>
#include <ArduinoEigenDense.h>


// Pinout
const uint8_t kPinReset = 17;
const uint8_t kPinInt = 22;
const uint8_t kPinCSEth = 15;
const uint8_t kPinMOSIEth = 13;
const uint8_t kPinMISOEth = 12;
const uint8_t kPinSCKEth = 14;
const uint8_t kPinIntEth = 27;
const uint8_t kPinResetEth = 26;

#if IMU_COMS == 0
  const uint8_t kPinSDA = 21;
  const uint8_t kPinSCL = 22;
#elif IMU_COMS == 1
  const uint8_t kPinMOSI = 23;
  const uint8_t kPinMISO = 19;
  const uint8_t kPinSCK = 18;
  const uint8_t kPinCS = 5;
#endif


// Hardware
const uint16_t kReportPeriod = 50;      // [ms]
const uint16_t kdataQueueSize_ = 10;
const IPAddress kIpDevice_ = IPAddress(192, 168, 1, 101);
const IPAddress kIpGateway_ = IPAddress(192, 168, 1, 100);
const IPAddress kSubnet_ = IPAddress(255, 255, 255, 0);
const IPAddress kIpDns_ = IPAddress(8, 8, 8, 8);
const byte kMacAddress_[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
const IPAddress kIpAgent_ = IPAddress(192, 168, 1, 100);
const uint16_t kPort_ = 8888;
const uint8_t kFreuencySPIEth_ = 3;  // [MHz]

#if IMU_COMS == 0
  const uint16_t kAddress = 0x4B;
  const uint32_t kfrequencyI2C = 400000;  // [Hz]
#elif IMU_COMS == 1
  const uint32_t kfrequencySPI = 3000000; // [Hz]
#endif


// Globals
std::shared_ptr<BNO08X> bno08x_;
std::shared_ptr<ENC28J60> eth_;
QueueHandle_t dataQueue_;
TaskHandle_t dataProcessingTask_;
TaskHandle_t rosSpinTask_;
TaskHandle_t rosTimeSyncTask_;
TaskHandle_t taskMaintainEthernet_;
std::shared_ptr<rcl_publisher_t> publisher_;
std::shared_ptr<rclc_executor_t> executor_;
std::shared_ptr<rclc_support_t> support_;
std::shared_ptr<rcl_allocator_t> allocator_;
std::shared_ptr<rcl_node_t> node_;
static micro_ros_utilities_memory_conf_t conf_;
std::shared_ptr<SPIClass> spiEth_ = std::make_shared<SPIClass>(HSPI);

#if IMU_COMS == 0
  std::shared_ptr<TwoWire> Wire = TwoWire(0);
#elif IMU_COMS == 1
  std::shared_ptr<SPIClass> spiImu = std::make_shared<SPIClass>(VSPI);
#endif


// Function Declarations
/// @brief Processes data from the BNO08x sensor.
/// @param pvParameters Unused.
static void processData(void *pvParameters);


/// @brief Calls ros spin function.
/// @param pvParameters Unused.
static void rosSpin(void *pvParameters);


/// @brief Syncs time with ROS host.
/// @param pvParameters Unused.
static void rosTimeSync(void *pvParameters);


// Micro ROS custom interface.
static bool transportOpen(struct uxrCustomTransport * transport) {
  return eth_->udpTransportOpen(transport);
}


static bool transportClose(struct uxrCustomTransport * transport) {
  return eth_->udpTransportClose(transport);
}


static size_t transportWrite(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
  return eth_->udpTransportWrite(transport, buf, len, err);
}


static size_t transportRead(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
  return eth_->udpTransportRead(transport, buf, len, timeout, err);
}



/// @brief Resumes the read sensor task.
void IRAM_ATTR reportReady(void) {
  bno08x_->taskWrapperReadReport(bno08x_.get());
}
    

void setup(void) {
  Serial.begin(115200);
  delay(3000);  // Delay for 3 seconds to allow for reprogramming.
  
  while (!Serial) {
    delay(100);
  }
  
  log_d("Serial Started");
  log_d("Starting UDP...");
  spiEth_->begin(kPinSCKEth, kPinMISOEth, kPinMOSIEth, kPinCSEth);
  eth_ = std::make_shared<ENC28J60>(kPinCSEth, kPinIntEth, kPinResetEth, spiEth_, kFreuencySPIEth_);
  byte mac[6];

  for (size_t i = 0; i < 6; i++) {
    mac[i] = kMacAddress_[i];
  }

  eth_->begin(kIpDevice_, kIpGateway_, kSubnet_, kIpDns_, mac, kIpAgent_, kPort_);
  log_d("UDP started");
  log_d("Constructing queue...");
  dataQueue_ = xQueueCreate(kdataQueueSize_, sizeof(ImuData));
  log_d("Queue constructed");

  auto passGate = [] (const BaseType_t result, const std::string string) {
    if (result != pdPASS) {
      log_e("%s", string.c_str());
      while (1);
    }
  };

  log_d("Starting data processing task...");

  BaseType_t success = xTaskCreate(processData,
                                  "dataProcessing",
                                  4096,
                                  nullptr,
                                  priorityLevels::PRIORITY_NORMAL,
                                  &dataProcessingTask_);
                      
  passGate(success, "Main: Failed to create dataProcessing task");
  log_d("Data processing task started");

  #if IMU_COMS == 0
    log_d("Starting I2C...");
    bool status = false;

    while (!status) {
      status = Wire.begin(kPinSDA, kPinSCL, kfrequencyI2C);

      if (!status) {
        delay(100);
      }
    }

    log_d("I2C started");

  #elif IMU_COMS == 1
    log_d("Starting SPI...");
    spiImu->begin(kPinSCK, kPinMISO, kPinMOSI, kPinCS);
    log_d("SPI started");

  #else
    log_E("Invalid IMU_COMS value! Must be 0 (I2C) or 1 (SPI)");
    while (1) {};
  #endif

  #if IMU_COMS == 0
    log_d("Starting BNO08x in I2C...");
    bno08x_ = std::make_shared<BNO08X>(kPinReset, 
                                      kPinInt, 
                                      Wire, 
                                      dataQueue_, 
                                      kReportPeriod, 
                                      kAddress);
    log_d("I2C BNO08x started");

  #elif IMU_COMS == 1
    log_d("Starting BNO08x in SPI...");
    bno08x_ = std::make_shared<BNO08X>(kPinReset, 
                                      kPinInt, 
                                      kPinCS, 
                                      spiImu, 
                                      dataQueue_, 
                                      kReportPeriod, 
                                      kfrequencySPI);
    log_d("SPI BNO08x started");
  #endif

  log_d("Attaching IMU interrupt...");
  attachInterrupt(digitalPinToInterrupt(kPinInt), reportReady, FALLING);
  log_d("IMU interrupt attached");
  log_d("Starting ROS...");

  auto passGateRos = [] (const rcl_ret_t result, const std::string string) {
    if (result != RCL_RET_OK) {
      log_e("%s", string.c_str());
      while (1);
    }
  };

  rmw_uros_set_custom_transport(false, 
                                NULL, 
                                transportOpen, 
                                transportClose, 
                                transportWrite, 
                                transportRead);

  set_microros_transports();
  allocator_ = std::make_shared<rcl_allocator_t>();
  support_ = std::make_shared<rclc_support_t>();
  node_ = std::make_shared<rcl_node_t>();
  executor_ = std::make_shared<rclc_executor_t>();
  publisher_ = std::make_shared<rcl_publisher_t>();
  *allocator_ = rcl_get_default_allocator();

  rcl_ret_t error;
  error = rclc_support_init(support_.get(), 
                            0, 
                            NULL, 
                            allocator_.get());

  passGateRos(error, "ROS support init failed");
  log_d("ROS support init success");

  error = rclc_node_init_default(node_.get(), 
                                "micro_ros_arduino_node", 
                                "", 
                                support_.get());

  passGateRos(error, "ROS node init failed");
  log_d("ROS node init success");

  error = rclc_publisher_init_default(publisher_.get(),
                                      node_.get(),
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),  // message type
                                      "micro_ros_arduino_node_publisher");

  passGateRos(error, "ROS publisher init failed");
  log_d("ROS publisher init success");

  error = rclc_executor_init(executor_.get(), 
                            &support_->context, 
                            1, 
                            allocator_.get());

  passGateRos(error, "ROS executor init failed");
  log_d("ROS executor init success");

  success = xTaskCreate(rosSpin,
                        "rosSpin",
                        4096,
                        nullptr,
                        priorityLevels::PRIORITY_HIGHEST,
                        &rosSpinTask_);
                      
  passGateRos(error, "Main: Failed to create dataProcessing task");
  log_d("ROS spin task started");

  success = xTaskCreate(rosTimeSync,
                        "rosTimeSync",
                        4096,
                        nullptr,
                        priorityLevels::PRIORITY_LOW,
                        &rosTimeSyncTask_);

  passGateRos(error, "Main: Failed to create dataProcessing task");
  log_d("ROS spin task started");
  log_d("Setup complete");
}

// Not used
void loop(void) {
  vTaskDelete(NULL); // Delete main loop task
}


static void processData(void *pvParameters) {
  log_d("main: Process data task started");
  sensor_msgs__msg__Imu imuMsg;
  imuMsg.header.frame_id = micro_ros_string_utilities_init("imu");
  ImuData data;

  for (;;) {
    // Wait for data
    if (xQueueReceive(dataQueue_, &data, portMAX_DELAY) != pdPASS) {
      continue;
    }
    
    // Fill message
    imuMsg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    imuMsg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    imuMsg.angular_velocity.x = data.angularVelocity.x();
    imuMsg.angular_velocity.y = data.angularVelocity.y();
    imuMsg.angular_velocity.z = data.angularVelocity.z();
    imuMsg.linear_acceleration.x = data.linearAcceleration.x();
    imuMsg.linear_acceleration.y = data.linearAcceleration.y();
    imuMsg.linear_acceleration.z = data.linearAcceleration.z();
    imuMsg.orientation.w = data.quaternion.w();
    imuMsg.orientation.x = data.quaternion.x();
    imuMsg.orientation.y = data.quaternion.y();
    imuMsg.orientation.z = data.quaternion.z();

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        size_t index = (i * 3) + j;
        imuMsg.angular_velocity_covariance[index] = data.covarianceAngular(i, j);
        imuMsg.linear_acceleration_covariance[index] = data.covarianceLinear(i, j);
        imuMsg.orientation_covariance[index] = data.covarianceQuaternion(i, j);
      }
    }
    
    if (rcl_publish(publisher_.get(), &imuMsg, NULL) != RCL_RET_OK) {
      log_e("Failed to publish IMU data");
    }
  }
}


static void rosSpin(void *pvParameters) {
  log_d("main: ROS spin task started");
  const TickType_t taskDelayPeriod = 100; // [ms].
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, taskDelayPeriod);
    rclc_executor_spin_some(executor_.get(), RCL_MS_TO_NS(100));
  }
}


static void rosTimeSync(void *pvParameters) {
  log_d("main: ROS time sync task started");
  const TickType_t taskDelayPeriod = 60000; // [ms].
  const int16_t timeout = 1000; // [ms]. 
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (rmw_uros_sync_session(timeout) != RMW_RET_OK) {
      log_e("ROS time sync failed");
    }

    vTaskDelayUntil(&lastWakeTime, taskDelayPeriod);
  }
}


