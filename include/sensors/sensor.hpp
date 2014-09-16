#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <config/config.hpp>
#include <queue>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include "networking/config_connection.hpp"
#include "synchronization/concurrent_queue.hpp"
#include "networking/config_connection.hpp"
#include "visensor/visensor_constants.hpp"
#include "visensor/visensor_datatypes.hpp"

namespace visensor {

// used to clean up memory in shared pointers
template<typename T>
struct array_deleter {
  void operator ()(T const * p) {
    delete[] p;
  }
};

struct Measurement {
  typedef boost::shared_ptr<Measurement> Ptr;

  Measurement()
      : data(),
        buffer_size(0),
        bytes_in_buffer(0),
        timestamp(0),
        timestamp_host(0) {
  }
  ~Measurement() {
  }

  boost::shared_ptr<uint8_t> data;
  uint32_t buffer_size;
  uint32_t bytes_in_buffer;
  uint64_t timestamp;
  uint64_t timestamp_host;
  uint32_t timestamp_fpga_counter; //raw fpga counter timestamp (1e-5 s)
};

struct SensorSettings {
  SensorSettings(SensorId::SensorId id, SensorType::SensorType type,
                 uint32_t measurementBufferSize, uint32_t numOfMsgsInPackage,
                 bool constBufferSize, uint32_t calibrationBufferSize)
      : sensor_id(id),
        type(type),
        measurementBufferSize(measurementBufferSize),
        numOfMsgsInPackage(numOfMsgsInPackage),
        constBufferSize(constBufferSize),
        calibrationBufferSize(calibrationBufferSize),
        active(false),
        rate(0) {
  }
  const SensorId::SensorId sensor_id;
  const SensorType::SensorType type;
  const uint32_t measurementBufferSize;
  const uint32_t numOfMsgsInPackage;
  const bool constBufferSize;
  const uint32_t calibrationBufferSize;
  bool active;
  uint32_t rate;
};

class Sensor {
 public:
  typedef boost::shared_ptr<Sensor> Ptr;
  typedef std::map<SensorId::SensorId, Sensor::Ptr> IdMap;

  Sensor(SensorSettings sensorSettings,
         ConfigConnection::WeakPtr config_connection);
  virtual ~Sensor();

  virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value)=0;
  virtual void init() = 0;
  virtual void setUserCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback){};
  virtual void setFrameCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback){};
  virtual void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback){};

  virtual void setUserCallback(boost::function<void(ViExternalTriggerMsg::Ptr)> callback){};
  virtual void setUserCallback(boost::function<void (ViFrame::Ptr, ViCorner::Ptr)> callback){};

  bool startSensor(uint32_t rate);
  bool stopSensor();
  uint64_t getTimeBetweenMsgs() const;
  int getNumOfMsgsInPackage() const;
  int getMeasurementBufferSize() const;
  Measurement getEmptyCalibrationMsg() const;
  void addMeasurement(const Measurement::Ptr meas);

  inline bool getSensorActive() const {
    return settings_.active;
  }

  inline bool getIsConstBufferSize() const {
    return settings_.constBufferSize;
  }

  inline SensorType::SensorType type() const {
    return settings_.type;
  }

  inline const SensorId::SensorId id() const {
    return settings_.sensor_id;
  }

  const SensorSettings& settings() const {
    return settings_;
  }

 protected:
  void writeRequest(const ViConfigMsg configMsg);
  void readRequest(ViConfigMsg& configMsg);
  bool checkTimestamp(uint64_t timestamp);
  ConfigConnection::Ptr getConfigConnection();

 public:
  concurrent_queue<std::queue<Measurement::Ptr> > measurement_queue_;

 private:
  //config server conenction
  ConfigConnection::WeakPtr config_connection_;

  SensorSettings settings_;
  uint64_t prev_timestamp_;
  uint32_t allowed_timediff_;
  uint32_t measurement_miss_counter_;
};

}  //namespace visensor

#endif /* SENSOR_HPP_ */
