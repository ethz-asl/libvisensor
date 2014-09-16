/*
 * camera_mt9v034.hpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#ifndef DENSE_MATCHER_HPP_
#define DENSE_MATCHER_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/param_server.hpp"
#include "sensors/camera.hpp"

namespace visensor {

namespace DenseMatcherDefaults {
const int I2C_ADRESS = 0x48,
    WIDTH = 752,
    HEIGHT = 480,
    RATE = CAMERA_FREQUENCY,
    NUM_OF_MSGS_IN_PACKAGE = 1,
    CALIBRATION_SIZE = 0;
const uint8_t COM_TYPE = ViComType::FPGA_32;

const ViImageType IMAGE_TYPE = ViImageType::MONO8;
const bool USE_CONST_PACKAGE_SIZE = true;
}

class DenseMatcherConfig {
 public:
  DenseMatcherConfig(const SensorId::SensorId sensorId)
      : sensor_id_(sensorId) {
    // name, type, register, default, mask, min, max
    param_server_.addParam("penalty_1", param_server::UINT_T, 0X04 * 40, 2, 0xFFFF, 0, 32765);
    param_server_.addParam("penalty_2", param_server::UINT_T, 0X04 * 41, 60, 0xFFFF, 0, 32765);
    param_server_.addParam("threshold", param_server::UINT_T, 0X04 * 42, 255, 0xFFFF, 0, 32765);
    param_server_.addParam("lr_check", param_server::UINT_T, 0X04 * 43, 5, 0xFFFF, 0, 32765);
  }

  ViConfigMsg getConfigParam(const std::string paramName, uint16_t value) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, value, msg.reg,
                                                  msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = DenseMatcherDefaults::I2C_ADRESS;
    msg.comType = DenseMatcherDefaults::COM_TYPE;
    return msg;
  }

  ViConfigMsg getConfigParam(const std::string paramName) {
    ViConfigMsg msg;
    msg.valChanged = param_server_.getConfigParam(paramName, msg.reg, msg.val);
    msg.sensorId = sensor_id_;
    msg.devAdress = DenseMatcherDefaults::I2C_ADRESS;
    msg.comType = DenseMatcherDefaults::COM_TYPE;
    return msg;
  }

  const SensorId::SensorId sensor_id_;

 private:
  param_server::ParamServer param_server_;
};

class DenseMatcher : public Camera {
 public:
  typedef boost::shared_ptr<DenseMatcher> Ptr;

  DenseMatcher(SensorId::SensorId sensor_id,
               ConfigConnection::WeakPtr config_connection);
  virtual ~DenseMatcher();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
  void setCalibration(ViCameraCalibration& calib_cam0, ViCameraCalibration& calib_cam1);

  //the threaded function that works on the queue of finished measurements
  void processMeasurements();
  virtual void init();
  static uint32_t calculateBufferSize();

 private:
  void bitReshuffle(uint8_t *in /*10 bit values*/,
                    uint8_t *out /*8 bit values*/,
                    int size_in /*number of 10 bit values*/, int offset);
  void imageReshuffle(uint8_t *data_in /*10 bit raw data*/, uint8_t *image_out,
                      int image_height, int image_width);

 private:
  uint32_t frame_size_; /* size of one frame */
  int width_;
  int height_;
  DenseMatcherConfig config_;
};

}  //namespace visensor

#endif /* CAMERA_MT9V034_HPP_ */
