#ifndef IMU_HPP_
#define IMU_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "sensors/sensor.hpp"

namespace visensor {
#define STANDARD_GRAVITY 9.80665

class Imu : public Sensor {
 public:
  typedef boost::shared_ptr<Imu> Ptr;

  Imu(SensorId::SensorId sensor_id, SensorSettings sensorSettings,
      ConfigConnection::WeakPtr config_connection)
      : Sensor(sensorSettings, config_connection),
        imu_id_(sensor_id) {
  }
  virtual ~Imu() {
  }
  ;
  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value)=0;

  void publishImuData(ViImuMsg::Ptr imu, ViErrorCode error);
  void setUserCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback);
  virtual void init() = 0;

 protected:
  boost::function<void(ViImuMsg::Ptr, ViErrorCode)> user_callback_;
  const SensorId::SensorId imu_id_;
};
}  //namespace visensor

#endif /* IMU_HPP_ */
