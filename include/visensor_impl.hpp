#ifndef VISENSOR_IMPL_HPP_
#define VISENSOR_IMPL_HPP_

#include <config/config.hpp>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "visensor/visensor.hpp"
#include "visensor/visensor_datatypes.hpp"
#include "visensor/visensor_exceptions.hpp"

#include "networking/connection.hpp"
#include "networking/file_transfer.hpp"
#include "synchronization/frame_corner_synchronizer.hpp"

// sensors
#include "sensors/sensor_factory.hpp"
#include "sensors/sensor.hpp"
#include "sensors/camera_mt9v034.hpp"
#include "sensors/camera_tau640.hpp"
#include "sensors/imu_adis16448.hpp"
#include "sensors/corner_mt9v034.hpp"


namespace visensor {

class ViSensorDriver::Impl {
 public:
  Impl();
  virtual ~Impl();

  void getAutoDiscoveryDeviceList(ViDeviceList &hostname_list);

  void init(std::string hostname);
  std::string initAutodiscovery();

  void startSensor(SensorId::SensorId sensor_id, uint32_t rate = CAMERA_FREQUENCY);
  void stopSensor(SensorId::SensorId sensor_id);
  void setSensorConfigParam(SensorId::SensorId sensor_id, const std::string cmd, uint16_t value);

  void startAllCameras(uint32_t rate = CAMERA_FREQUENCY);
  void setCameraCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  void startAllCorners();
  void setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback);

  void startAllImus(uint32_t rate = IMU_FREQUENCY);
  void setImuCallback(boost::function<void(ViImuMsg::Ptr, ViErrorCode)> callback);

  void startAllExternalTriggers(uint32_t rate);
  void setExternalTriggerCallback(
      boost::function<void(ViExternalTriggerMsg::Ptr)> callback);
  void setExternalTriggerConfig(const ViExternalTriggerConfig config);

  void startAllDenseMatchers();
  void setDenseMatcherCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback);

  bool getCameraCalibration(SensorId::SensorId cam_id, ViCameraCalibration &calib);
  bool getCameraCalibration(SensorId::SensorId cam_id, int slot_id, ViCameraCalibration &calib);
  bool setCameraCalibration(SensorId::SensorId cam_id, int slot_id, const ViCameraCalibration calib);
  bool setCameraFactoryCalibration(SensorId::SensorId cam_id, const ViCameraCalibration calib);

  // is called with synchronized images and corresponding corners
  void setFramesCornersCallback(
      boost::function<
          void(ViFrame::Ptr, ViCorner::Ptr)> callback);

  std::vector<SensorId::SensorId> getListOfSensorIDs() const;
  std::vector<SensorId::SensorId> getListOfCameraIDs() const;
  std::vector<SensorId::SensorId> getListOfDenseIDs() const;
  std::vector<SensorId::SensorId> getListOfCornerIDs() const;
  std::vector<SensorId::SensorId> getListOfImuIDs() const;
  std::vector<SensorId::SensorId> getListOfTriggerIDs() const;
  uint32_t getFpgaId() const;

  void setCameraCalibrationSlot(int slot_id = 0); // slot 0 is factory calibration
  int getCameraCalibrationSlot();
  void downloadFile(std::string& local_path, std::string& remote_path);
  void uploadFile(std::string& local_path, std::string& remote_path);

  void sendSerialData(ViSerialData::Ptr data);
  void setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback);
  void setSerialDelimiter(const char serial_id, const std::string delimiter);
  void setSerialBaudrate(const char serial_id, const unsigned int baudrate);

 private:
  bool isInitialized();

  IpConnection::Ptr ip_connection_;
  SerialHost::Ptr serial_host_;

  Sensor::IdMap sensors_;
  boost::thread_group sensor_threads_;

  bool initialized_;

  int current_camera_calibration_slot_;

  std::vector<FrameCornerSynchronizer*> cam_corner_syncronizer_;
};
}  //namespace visensor

#endif
