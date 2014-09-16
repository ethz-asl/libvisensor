#ifndef Corner_DETECTOR_HPP_
#define Corner_DETECTOR_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "sensors/sensor.hpp"

namespace visensor {

struct ViCornerConfig {
  ViCornerConfig(uint32_t width_, uint32_t height_, uint32_t frame_rate_)
      : width(width_),
        height(height_),
        frame_rate(frame_rate_) {
  }
  ;
  uint32_t width; /* the image width */
  uint32_t height; /* the image height */
  uint32_t frame_rate; /* the Corner frame rate */
};

class CornerDetector : public Sensor {
 public:
  typedef boost::shared_ptr<CornerDetector> Ptr;

  CornerDetector(SensorId::SensorId sensor_id, ViCornerConfig config,
                 SensorSettings sensorSettings,
                 ConfigConnection::WeakPtr config_connection)
      : Sensor(sensorSettings, config_connection),
        config_(config),
        camera_id_(sensor_id){
  }
  ;

  virtual ~CornerDetector() {
  }
  ;

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value) = 0;
  void publishCornerData(ViCorner::Ptr& frame, ViErrorCode error);
  void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback);
  //the threaded function that works on the queue of finished measurements
  virtual void processMeasurements() = 0;
  virtual void init() = 0;

 protected:
  boost::function<void(ViCorner::Ptr, ViErrorCode)> user_callback_;
  ViCornerConfig config_; /* the Corner configuration */
  const uint32_t camera_id_;
};
}  //namespace visensor

#endif /* Corner_HPP_ */
