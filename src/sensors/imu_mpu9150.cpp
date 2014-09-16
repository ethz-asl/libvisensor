#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include <sensors/imu_mpu9150.hpp>

namespace visensor {

ImuMpu9150::ImuMpu9150(SensorId::SensorId sensor_id,
                       IpConnection::WeakPtr config_connection)
    : Imu(sensor_id,
          SensorSettings(sensor_id, SensorType::SensorType::MPU_9150,
                         calculateBufferSize(),
                         ImuMpu9150Defaults::NUM_OF_MSGS_IN_PACKAGE,
                         ImuMpu9150Defaults::USE_CONST_PACKAGE_SIZE,
                         ImuMpu9150Defaults::CALIBRATION_SIZE),
          config_connection),
      config_(sensor_id) {
}

uint32_t ImuMpu9150::calculateBufferSize() {
  return (ImuMpu9150Defaults::MSG_SIZE);
}

void ImuMpu9150::init() {
  writeRequest(config_.getConfigParam("enable"));
  writeRequest(config_.getConfigParam("digital_low_pass_filter_config"));
}

void ImuMpu9150::processMeasurements() {
  while (1) {
    boost::this_thread::interruption_point();

    //get the newest measurement
    boost::shared_ptr<Measurement> meas = Sensor::measurement_queue_.pop();

    //checkTimestamp(meas.timestamp);

    ViImuMsg::Ptr imu_msg_ptr = boost::make_shared<ViImuMsg>();

    // Add imu information to the msg
    imu_msg_ptr->imu_id = Imu::imu_id_;
    imu_msg_ptr->timestamp = meas->timestamp;
    getGyro(meas->data.get(), &imu_msg_ptr->gyro[0]);
    getTemperature(meas->data.get(), &imu_msg_ptr->temperature);
    getAcc(meas->data.get(), &imu_msg_ptr->acc[0]);

    publishImuData(imu_msg_ptr, ViErrorCodes::NO_ERROR);
  }
}

ViConfigMsg ImuMpu9150::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

void ImuMpu9150::getAcc(uint8_t* buffer, double * acc) {
  int i, current_pos;
  int16_t temp;

  for (i = 0; i < 3; i++) {
    current_pos = 2 * i;
    temp = (int) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
    acc[i] = (double) temp;
    acc[i] = acc[i] / 16384 * STANDARD_GRAVITY;
  }
}

void ImuMpu9150::getGyro(uint8_t* buffer, double * gyro) {
  int i, current_pos;
  int16_t temp;
  for (i = 0; i < 3; i++) {
    current_pos = 2 * 3 + 2 + 2 * i;
    temp = (int) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
    gyro[i] = (double) temp;
    gyro[i] = gyro[i] / 131 * M_PI / 180;
  }
}

void ImuMpu9150::getTemperature(uint8_t* buffer, double * temperature) {
  int current_pos;
  int16_t temp;
  current_pos = 2 * 3;
  temp =
      (int16_t) ((buffer[current_pos] << 8) | (buffer[current_pos + 1] << 0));
  *temperature = (double) temp;
  *temperature = *temperature / 340.0 + 35.0;
}

}  //namespace visensor
