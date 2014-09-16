#ifndef IP_CONNECTION_
#define IP_CONNECTION_

#include <config/config.hpp>

#include <vector>
#include <map>
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>

#include "visensor/visensor_datatypes.hpp"
#include "synchronization/time_synchronizer.hpp"
#include "networking/ip_data_definitions.hpp"
#include "networking/file_transfer.hpp"
#include "networking/config_connection.hpp"
#include "serial_bridge/SerialHost.hpp"
#include "sensors/sensor.hpp"

namespace visensor {

//Fpga version struct
struct FpgaConfig {
  FpgaConfig() {
    fpga_id_ = 0;
    firmware_version_major = 0;
    firmware_version_minor = 0;
    firmware_version_patch = 0;
    num_of_sensors_ = 0;
  }
  uint32_t fpga_id_;
  uint16_t firmware_version_major;
  uint16_t firmware_version_minor;
  uint16_t firmware_version_patch;
  uint16_t num_of_sensors_;
};

class IpConnection : public visensor::ConfigConnection {
 public:
  typedef boost::shared_ptr<IpConnection> Ptr;
  typedef boost::weak_ptr<IpConnection> WeakPtr;

  IpConnection();
  ~IpConnection();

  void connect(std::string hostname);
  bool sendLedConfig(uint16_t value);
  // set lightning mode:
  // 0 - strobe
  // 1 - continuous
  bool sendLedConfig(bool value);

  void sendExternalTriggerConfig(SensorId::SensorId sensor_id, const ViExternalTriggerConfig config);

  void addSensor(SensorId::SensorId sensor_id, Sensor::Ptr sensor);
  void startSensor(SensorId::SensorId sensor_id, uint32_t rate);
  void stopSensor(SensorId::SensorId sensor_id);

  bool readCameraCalibration(SensorId::SensorId sensor_id, unsigned int slot_id, ViCameraCalibration &calib_out);
  bool writeCameraCalibration(SensorId::SensorId sensor_id, unsigned int slot_id, const ViCameraCalibration calib);

  //serial bridge communication
  void registerSerialHost(SerialHost::WeakPtr serial_host);
  SerialHost::Ptr getSerialHostPointer();

  void sendSerialData(ViSerialData::Ptr data_ptr);
  void setSerialDelimiter(const char sensor_id, const std::string delimiter);
  void setSerialBaudrate(const char sensor_id, const unsigned int baudrate);

  uint32_t getId() const {
    return fpga_config_.fpga_id_;
  }

  bool fpgaInitialized() const {
    return connected_;
  }

  virtual void writeConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                           uint32_t val, uint8_t comType);
  virtual void readConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                          uint32_t &val, uint8_t comType);

  void downloadFile(std::string& local_path, std::string& remote_path);
  void uploadFile(std::string& local_path, std::string& remote_path);

  std::vector<IpComm::SensorInfo> getAttachedSensors();

 private:
  void readFpgaInfo();
  void syncTime();

  bool sendHeader(int identifier);
  void sendStartSensor(SensorId::SensorId sensor_id, uint32_t rate);
  void sendStopSensor(SensorId::SensorId sensor_id);

  template<typename T>
  bool writeDataUbi(SensorId::SensorId sensor_id, unsigned char reg, T val,
                    int numBits);
  template<typename T>
  bool readDataUbi(SensorId::SensorId sensor_id, unsigned char reg, T *val,
                   int numBits);
  bool writeDataFpga(SensorId::SensorId sensor_id, unsigned char reg, int val);
  bool readDataFpga(SensorId::SensorId sensor_id, unsigned char reg, int &val);
  bool readInitMsg(unsigned char * msg);

  uint32_t getTimestampFpgaRaw(uint8_t* buffer);
  uint64_t getTimestamp(uint8_t* buffer);

  void processPackage(SensorId::SensorId sensor_id, Measurement::Ptr new_measurement);
  void read_handler(const boost::system::error_code& e,
                    std::size_t bytes_transferred);
  void imu_read_handler(const boost::system::error_code& e,
                        std::size_t bytes_transferred);
  void serial_read_handler(const boost::system::error_code& e,
                        std::size_t bytes_transferred);

  bool receiveAck(boost::asio::ip::tcp::socket &socket);

  template<class T>
  void send_package(IpComm::Header &header, T &data);
  template<class T, class P>
  void receive_package(IpComm::Header &header, T &data,
                       P payload_type /*only used to know the datatype*/);
  template<typename T, size_t N>
  void send_payload(boost::array<T, N>& data);
  template<typename T, size_t N>
  void receive_payload(boost::asio::ip::tcp::socket &socket, boost::array<T, N>& data);

  IpComm::Header readHeader(boost::asio::ip::tcp::socket &socket);

 private:
  bool connected_;
  TimeSynchronizer time_synchronizer_;
  uint8_t number_of_sensors_;
  std::map<SensorId::SensorId, Sensor::Ptr> sensors_;
  SensorId::SensorId led_sensor_id_;  // HACK: either add the light to the cam options or create a led sensor class
  FpgaConfig fpga_config_;

  //pointer to serialhost
  SerialHost::WeakPtr serial_host_;

  // connection variables
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket data_socket_;
  boost::asio::ip::tcp::socket imu_socket_;
  boost::asio::ip::tcp::socket serial_socket_;
  boost::asio::ip::tcp::socket config_socket_;
  IpComm::HeaderPayload data_header_payload_;
  IpComm::HeaderPayload imu_header_payload_;
  IpComm::HeaderPayload serial_header_payload_;

  FileTransfer file_transfer_;
};
}  //namespace visensor

#endif /* IP_CONNECTION_ */
