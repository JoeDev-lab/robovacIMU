#ifndef DATA_HPP
#define DATA_HPP

#include <ArduinoEigenDense.h>

// Data
enum priorityLevels {
  PRIORITY_LOWEST = 0,
  PRIORITY_LOW = 1,
  PRIORITY_NORMAL = 2,
  PRIORITY_HIGH = 3,
  PRIORITY_HIGHEST = 4
};


struct ImuData {
  Eigen::Quaternionf quaternion;
  Eigen::Matrix3f covarianceQuaternion;
  Eigen::Vector3f angularVelocity;
  Eigen::Matrix3f covarianceAngular;
  Eigen::Vector3f linearAcceleration;
  Eigen::Matrix3f covarianceLinear;

  ImuData(const Eigen::Quaternionf quaternionIn = Eigen::Quaternionf(),
          const Eigen::Matrix3f covarianceQuaternionIn = Eigen::Matrix3f(),
          const Eigen::Vector3f angularVelocityIn = Eigen::Vector3f(), 
          const Eigen::Matrix3f covarianceAngularIn = Eigen::Matrix3f(),
          const Eigen::Vector3f linearAccelerationIn = Eigen::Vector3f(),
          const Eigen::Matrix3f covarianceLinearIn = Eigen::Matrix3f()) 
                  : quaternion(quaternionIn), 
                    covarianceQuaternion(covarianceQuaternionIn), 
                    angularVelocity(angularVelocityIn), 
                    covarianceAngular(covarianceAngularIn), 
                    linearAcceleration(linearAccelerationIn),
                    covarianceLinear(covarianceLinearIn) {}
};

#endif // DATA_HPP
