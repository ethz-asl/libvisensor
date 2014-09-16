#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include <config/config.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>

#include "sensor.hpp"

namespace visensor {

struct ViCameraConfig {
  ViCameraConfig(uint32_t width_, uint32_t height_, uint32_t frame_rate_)
      : width(width_),
        height(height_),
        frame_rate(frame_rate_)
  {};

  uint32_t width; /* the image width */
  uint32_t height; /* the image height */
  uint32_t frame_rate; /* the camera frame rate */
};

class Camera : public Sensor {
 public:
  Camera(SensorId::SensorId sensor_id, ViCameraConfig config,
         SensorSettings sensorSettings,
         ConfigConnection::WeakPtr config_connection)
      : Sensor(sensorSettings, config_connection),
        config_(config),
        camera_id_(sensor_id) {};

  virtual ~Camera() {};

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value) = 0;
  void publishFrameData(ViFrame::Ptr& frame, ViErrorCode error);
  void setFrameCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  //the threaded function that works on the queue of finished measurements
  virtual void processMeasurements() = 0;
  virtual void init() = 0;

  typedef boost::shared_ptr<Camera> Ptr;

 public:
  ViCameraConfig config_; /* the camera configuration */

 protected:
  std::vector<boost::function<void(ViFrame::Ptr, ViErrorCode)> > user_callbacks_;
  const SensorId::SensorId camera_id_;
};
}  //namespace visensor

#endif /* CAMERA_HPP_ */
