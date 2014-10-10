/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include "visensor/visensor_version.hpp"
#include "visensor/visensor_exceptions.hpp"
#include "networking/connection.hpp"
#include "networking/file_transfer.hpp"


using boost::asio::ip::tcp;

namespace visensor
{
IpConnection::IpConnection()
: connected_(false),
  data_socket_(io_service_),
  imu_socket_(io_service_),
  serial_socket_(io_service_),
  config_socket_(io_service_),
  file_transfer_(config_socket_)
{}

IpConnection::~IpConnection(){
  VISENSOR_DEBUG("connection closed\n");
}

void IpConnection::connect(std::string sensor_address) {
  boost::asio::ip::tcp::resolver resolver(io_service_);

  try
  {
    tcp::resolver::query query_config(sensor_address, "13778");
    tcp::resolver::iterator endpoint_config = resolver.resolve(query_config);
    config_socket_.connect(*endpoint_config);
  }
  catch (boost::system::system_error const& e)
  {
    throw visensor::exceptions::ConnectionException("Could not connect to sensor at " + sensor_address);
  }

  if (!config_socket_.is_open())
    throw visensor::exceptions::ConnectionException("Could not connect to config stream at " + sensor_address);

  try {
    // open data connection
    tcp::resolver::query query_data(sensor_address, "13777");
    tcp::resolver::iterator endpoint_data= resolver.resolve(query_data);
    data_socket_.connect(*endpoint_data);

    tcp::resolver::query query_imu(sensor_address, "13776");
    tcp::resolver::iterator endpoint_imu= resolver.resolve(query_imu);
    imu_socket_.connect(*endpoint_imu);

    tcp::resolver::query query_serial(sensor_address, "13780");
    tcp::resolver::iterator endpoint_serial = resolver.resolve(query_serial);
    serial_socket_.connect(*endpoint_serial);
  } catch (boost::system::system_error const& e) {
    VISENSOR_DEBUG("Connection could not be established!\n");
    throw visensor::exceptions::ConnectionException("Could not connect to data sockets at " + sensor_address);
  }

  //check all data sockets (have to be open, if the config connected successfully)
  VISENSOR_ASSERT_COND(!data_socket_.is_open(), "Can't connect camera data socket.\n");
  VISENSOR_ASSERT_COND(!imu_socket_.is_open(), "Can't connect imu data socket.\n");
  VISENSOR_ASSERT_COND(!serial_socket_.is_open(), "Can't connect serial data socket.\n");

  VISENSOR_DEBUG("Connection established\n");

  //read fpga info
  readFpgaInfo();
  connected_=true;

  //synchronize clocks
  syncTime();

  // start receiving data streams
  async_read(data_socket_, boost::asio::buffer(data_header_payload_), boost::asio::transfer_all(),
             boost::bind(&IpConnection::read_handler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));
  // start receiving imu streams
  async_read(imu_socket_, boost::asio::buffer(imu_header_payload_), boost::asio::transfer_all(),
             boost::bind(&IpConnection::imu_read_handler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));

  // start receiving serial streams
  async_read(serial_socket_, boost::asio::buffer(serial_header_payload_), boost::asio::transfer_all(),
             boost::bind(&IpConnection::serial_read_handler, this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred));

  boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service_));
}

void IpConnection::registerSerialHost(SerialHost::WeakPtr serial_host)
{
  serial_host_ = serial_host;
}

SerialHost::Ptr IpConnection::getSerialHostPointer()
{
  VISENSOR_ASSERT_COND(serial_host_._empty(), "Register serial host pointer first...!");
  return serial_host_.lock();
}

void IpConnection::sendSerialData(ViSerialData::Ptr data_ptr)
{
  //continue only if we have a client connected
  if(!connected_)
    return;

  //send data over network
  IpComm::Header header;
  header.timestamp = 0;
  header.data_size = data_ptr->data.size() + 1; //first byte is port_id (char)
  header.data_id = IpComm::SERIAL_DATA;

  boost::asio::write(serial_socket_, boost::asio::buffer(header.getSerialized()));

  //write payload (sensor_id + serial_data)
  boost::asio::write(serial_socket_, boost::asio::buffer(&data_ptr->port_id, 1));
  boost::asio::write(serial_socket_, boost::asio::buffer(data_ptr->data));
}

void IpConnection::setSerialDelimiter(const char serial_id, const std::string delimiter)
{
  //continue only if we have a client connected
  if(!connected_)
    return;

  //send data over network
  IpComm::Header header;
  header.timestamp = 0;
  header.data_size = delimiter.size() + 1; //first byte is port_id (char)
  header.data_id = IpComm::SERIAL_SET_DELIMITER;

  boost::asio::write(serial_socket_, boost::asio::buffer(header.getSerialized()));

  //write payload (sensor_id + serial_data)
  boost::asio::write(serial_socket_, boost::asio::buffer(&serial_id, 1));
  boost::asio::write(serial_socket_, boost::asio::buffer(delimiter));
}

void IpConnection::setSerialBaudrate(const char serial_id, const unsigned int baudrate)
{
  //continue only if we have a client connected
  if(!connected_)
    return;

  //send data over network
  IpComm::Header header;
  header.timestamp = 0;
  header.data_size = 1 + 4; //first byte is port_id (char) + baudrate (uint32)
  header.data_id = IpComm::SERIAL_SET_BAUDRATE;

  boost::asio::write(serial_socket_, boost::asio::buffer(header.getSerialized()));

  //write payload (sensor_id + serial_data)
  boost::asio::write(serial_socket_, boost::asio::buffer(&serial_id, 1));
  boost::asio::write(serial_socket_, boost::asio::buffer((char*)&baudrate,4));
}

void IpConnection::addSensor(SensorId::SensorId sensor_id, Sensor::Ptr sensor) {
  sensors_.insert(std::pair<SensorId::SensorId, Sensor::Ptr>(sensor_id, sensor));
}

void IpConnection::syncTime(){

  VISENSOR_DEBUG("syncing time \n");
  sendHeader(IpComm::SYNC_TIME);

  int number_of_measurements = 10;
  std::vector<uint64_t> time_differences(number_of_measurements);
  std::vector<uint64_t> fpga_time(number_of_measurements);
  IpComm::TimeSync ping;
  IpComm::TimeSyncPayload pong_payload;
  uint64_t total_time = 0;

  // send one ping pong to wait for free network
  ping.host_time = 1;
  ping.fpga_time = 0;
  boost::asio::write(config_socket_, boost::asio::buffer(ping.getSerialized()));
  receive_payload(config_socket_, pong_payload);

  uint64_t minRoundTrip = 1e9;

  // run the measurements
  for(int i=0;i<number_of_measurements;++i)
  {
    // send ping to sensor
    ping.host_time = time_synchronizer_.getSystemTime();
    boost::asio::write(config_socket_, boost::asio::buffer(ping.getSerialized()));

    // receive pong from sensor
    receive_payload(config_socket_, pong_payload);
    IpComm::TimeSync pong(pong_payload);

    // save time difference
    time_differences[i] = time_synchronizer_.getSystemTime() - pong.host_time;
    fpga_time[i] = pong.fpga_time;

    total_time += time_differences[i];
    //VISENSOR_DEBUG("time_differences: %llu\n", time_differences[i]);
    VISENSOR_DEBUG(".");

    if(time_differences[i] < minRoundTrip)
    {
        time_synchronizer_.init((pong.host_time + time_differences[i]/2), (fpga_time[i]*10e3));
        minRoundTrip = time_differences[i];
    }
  }

  // send the termination signal
  ping.host_time = 0;
  boost::asio::write(config_socket_, boost::asio::buffer(ping.getSerialized()));


  double average = (double)total_time/(double)number_of_measurements/1000000.0;
  VISENSOR_DEBUG(" average: %fms\n", average);
}

void IpConnection::read_handler(const boost::system::error_code& error,
                                    std::size_t bytes_transferred) {
  if (!error)
  {
    VISENSOR_ASSERT_COND(bytes_transferred != sizeof(data_header_payload_),
                    "not correct number of bytes received: %lu != %lu\n",
                    bytes_transferred,
                    sizeof(data_header_payload_));


    IpComm::Header header(data_header_payload_);

    // VISENSOR_DEBUG("header: %d, %d, %d\n", header.timestamp, header.data_size, header.data_id);

    // allocate memory for new data
    uint8_t timestamp_buffer[4];

    uint8_t* data_ptr = new uint8_t[header.data_size-4];

    // build receive buffers pointing to allocated memory
    std::vector<boost::asio::mutable_buffer> receive_buffers;
    receive_buffers.push_back(boost::asio::buffer(timestamp_buffer, 4));
    receive_buffers.push_back(boost::asio::buffer(data_ptr, header.data_size-4));

    // receive the data
    boost::system::error_code receive_error;
    unsigned int nBytesReceived = boost::asio::read(data_socket_, receive_buffers, boost::asio::transfer_all(), receive_error);

    boost::shared_ptr<uint8_t> data( data_ptr, array_deleter<uint8_t>() );

    if(receive_error)
    {
      // handle errors which happen when terminating the code
      if (receive_error != boost::asio::error::operation_aborted
          || receive_error != boost::asio::error::bad_descriptor)
      {
        VISENSOR_DEBUG("Camera server: receive handler aborted due to client shutdown\n");
        return;
      }
    }

    VISENSOR_ASSERT_COND(nBytesReceived != header.data_size,
                    "not correct number of bytes received: %u != %u\n",
                    nBytesReceived,
                    header.data_size);

    Measurement::Ptr measurement = boost::make_shared<Measurement>();
    measurement->timestamp = getTimestamp(timestamp_buffer);
    measurement->timestamp_fpga_counter = getTimestampFpgaRaw(timestamp_buffer)  ; //raw fpga counter timestamp
    measurement->data = data;

    measurement->buffer_size = header.data_size-4;

    processPackage(static_cast<SensorId::SensorId>(header.data_id), measurement);

    // ready to receive a new data header
    async_read(data_socket_, boost::asio::buffer(data_header_payload_), boost::asio::transfer_all(),
                                boost::bind(&IpConnection::read_handler, this, boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
  }
  else if (error == boost::asio::error::operation_aborted || error != boost::asio::error::bad_descriptor)
  {
    // handle errors which happen when terminating the code
    VISENSOR_DEBUG("Camera server: receive handler aborted due to client shutdown\n");
    return;
  }
  else
    throw visensor::exceptions::ConnectionException("Connection to sensor lost.");
}

void IpConnection::imu_read_handler(const boost::system::error_code& error,
                                    std::size_t bytes_transferred) {
  if (!error)
  {
    VISENSOR_ASSERT_COND(bytes_transferred != sizeof(imu_header_payload_),
                    "not correct number of bytes received: %lu != %lu\n",
                    bytes_transferred,
                    sizeof(imu_header_payload_));

    IpComm::Header header(imu_header_payload_);

//    VISENSOR_DEBUG("header: %d, %d, %d\n", header.timestamp, header.data_size, header.data_id);

    // read payload
    uint8_t timestamp_buffer[4];
    uint8_t* data_ptr = new uint8_t[header.data_size-4];

    std::vector<boost::asio::mutable_buffer> receive_buffers;
    receive_buffers.push_back(boost::asio::buffer(timestamp_buffer, 4));
    receive_buffers.push_back(boost::asio::buffer(data_ptr, header.data_size-4));
    boost::system::error_code receive_error;
    unsigned int nBytesReceived = boost::asio::read(imu_socket_, receive_buffers, boost::asio::transfer_all(), receive_error);

    boost::shared_ptr<uint8_t> data( data_ptr, array_deleter<uint8_t>() );

    if(receive_error)
    {
      // handle errors which happen when terminating the code
      if (receive_error != boost::asio::error::operation_aborted
          || receive_error != boost::asio::error::bad_descriptor)
      {
        VISENSOR_DEBUG("Imu server: receive handler aborted due to client shutdown\n");
        return;
      }
    }

    VISENSOR_ASSERT_COND(nBytesReceived != header.data_size,
                    "not correct number of bytes received: %u != %u\n",
                    nBytesReceived,
                    header.data_size);

    // fill data into measurement
    Measurement::Ptr measurement( new Measurement() );
    measurement->timestamp = getTimestamp(timestamp_buffer);
    measurement->timestamp_fpga_counter = getTimestampFpgaRaw(timestamp_buffer)  ; //raw fpga counter timestamp
    measurement->data = data;
    measurement->buffer_size = header.data_size-4;

    processPackage(static_cast<SensorId::SensorId>(header.data_id), measurement);

    async_read(imu_socket_, boost::asio::buffer(imu_header_payload_), boost::asio::transfer_all(),
                                boost::bind(&IpConnection::imu_read_handler, this, boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
  }
  else if (error == boost::asio::error::operation_aborted || error != boost::asio::error::bad_descriptor)
  {
    // handle errors which happen when terminating the code
    VISENSOR_DEBUG("Imu server: receive handler aborted due to client shutdown\n");
    return;
  }
  else
    throw visensor::exceptions::ConnectionException("Connection to sensor lost.");
}


void IpConnection::serial_read_handler(const boost::system::error_code& error,
                                    std::size_t bytes_transferred) {
  if (!error)
  {
    VISENSOR_ASSERT_COND(bytes_transferred != sizeof(serial_header_payload_),
                    "not correct number of bytes received: %lu != %lu\n",
                    bytes_transferred,
                    sizeof(serial_header_payload_));

    //receive payload
    boost::system::error_code receive_error;
    IpComm::Header header(serial_header_payload_);
    std::vector<char> body_payload;
    body_payload.resize(header.data_size);
    boost::asio::read(serial_socket_, boost::asio::buffer(body_payload), boost::asio::transfer_all(), receive_error);

    if(receive_error)
    {
      // handle errors which happen when terminating the code
      if (receive_error != boost::asio::error::operation_aborted
          || receive_error != boost::asio::error::bad_descriptor)
      {
        VISENSOR_DEBUG("Serial server: receive handler aborted due to client shutdown\n");
        return;
      }
    }

    switch(header.data_id)
    {
     case IpComm::SERIAL_DATA:
     {
       ViSerialData::Ptr msg_ptr = boost::make_shared<ViSerialData>();
       msg_ptr->direction = visensor::ViSerialData::FROM_SERIAL_DEVICE;
       msg_ptr->port_id = (uint32_t)body_payload[0];//first byte is the serial port id
       msg_ptr->data = std::string(body_payload.begin()+1, body_payload.end());

       //add to publishing queue
       if(!serial_host_._empty())
         getSerialHostPointer()->addDataToPublishQueue(msg_ptr);

       //VISENSOR_DEBUG("serial port %u: %s", msg_ptr->port_id, msg_ptr->data.c_str());
       break;
     }
     default:
     {
       VISENSOR_DEBUG("Received unknown header id %u on serial socket...!\n", header.data_id);
       break;
     }
    }

    //wait for the next header
    async_read(serial_socket_, boost::asio::buffer(serial_header_payload_), boost::asio::transfer_all(),
                                boost::bind(&IpConnection::serial_read_handler, this, boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
  }
  else if (error == boost::asio::error::operation_aborted || error != boost::asio::error::bad_descriptor)
  {
    // handle errors which happen when terminating the code
    VISENSOR_DEBUG("Serial server: receive handler aborted due to client shutdown\n");
    return;
  }
  else
    throw visensor::exceptions::ConnectionException("Connection to sensor lost."); // Some other error.
}

bool IpConnection::sendHeader(int identifier) {
  IpComm::Header header;
  header.data_id = identifier;
  header.data_size = 0;
  header.timestamp = 0;

  boost::asio::write(config_socket_,boost::asio::buffer(header.getSerialized()));
  return true;
}

void IpConnection::sendStartSensor(SensorId::SensorId sensor_id, uint32_t rate) {
  // send start package header + payload (id+rate)
  IpComm::Header header;
  header.data_id = IpComm::START_SENSOR;
  header.data_size = 1;
  header.timestamp = 0;

  IpComm::StartSensor package;
  package.id = sensor_id;
  package.rate = rate;

  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(header.getSerialized()));
  buffers.push_back(boost::asio::buffer(package.getSerialized()));
  boost::asio::write(config_socket_, buffers);
}

void IpConnection::sendStopSensor(SensorId::SensorId sensor_id) {
  IpComm::Header header;
  header.data_id = IpComm::STOP_SENSOR;
  header.data_size = 0;
  header.timestamp = sensor_id;

  boost::asio::write(config_socket_,boost::asio::buffer(header.getSerialized()));
}

template<typename T, size_t N>
inline void visensor::IpConnection::send_payload(
    boost::array<T, N>& data) {
  boost::asio::write(config_socket_, boost::asio::buffer(data));
}

template<typename T, size_t N>
inline void visensor::IpConnection::receive_payload(boost::asio::ip::tcp::socket &socket,
                                                    boost::array<T, N>& data) {
  boost::asio::read(socket, boost::asio::buffer(data));
}

template<class T>
inline void IpConnection::send_package(IpComm::Header &header,
                                                      T &data) {
  // send at once screwed up the data order
  boost::asio::write(config_socket_, boost::asio::buffer(header.getSerialized()));
  boost::asio::write(config_socket_, boost::asio::buffer(data.getSerialized()));
}

template<class T, class P>
inline void IpConnection::receive_package(
    IpComm::Header &header, T &data, P payload_type) {

  IpComm::HeaderPayload headerInPayload;
  P dataInPayload;

  std::vector<boost::asio::mutable_buffer> buffersIn;
  buffersIn.push_back(boost::asio::buffer(headerInPayload));
  buffersIn.push_back(boost::asio::buffer(dataInPayload));
  boost::asio::read(config_socket_, buffersIn);

  header.setSerialized(headerInPayload);
  data.setSerialized(dataInPayload);
}

bool IpConnection::receiveAck(boost::asio::ip::tcp::socket &socket)
{
  IpComm::Header response = readHeader(socket);
  if(response.data_id == IpComm::ACK)
    return true;
  else
    return false;
}

void IpConnection::downloadFile(std::string& local_path, std::string& remote_path) {
  // send the file request
  sendHeader(IpComm::REQUEST_FILE);

  // receive file and write to path
  file_transfer_.receiveFile(local_path, remote_path);
}

void IpConnection::uploadFile(std::string& local_path, std::string& remote_path) {
  // prepare sensor to receive calibration file
  sendHeader(IpComm::SEND_FILE);

  // read file from path and send it to the sensor
  file_transfer_.sendFile(local_path, remote_path);
}

IpComm::Header IpConnection::readHeader(boost::asio::ip::tcp::socket &socket) {
  IpComm::HeaderPayload header_payload;
  receive_payload(socket, header_payload);
  IpComm::Header header(header_payload);
  return header;
}

void IpConnection::readFpgaInfo()
{
  sendHeader(IpComm::REQUEST_FPGA_INFO);

  IpComm::Header header = readHeader(config_socket_);
  if(header.data_id != IpComm::FPGA_INFO)
    throw visensor::exceptions::FirmwareException("FPGA_INFO request failed. Check firmware version.");

  IpComm::FpgaInfoPayload fpga_info_payload;
  if(header.data_size == 0)
    throw visensor::exceptions::FirmwareException("Old sensor firmware detected, please use the visensor-update tool to update the firmware.");

  receive_payload(config_socket_, fpga_info_payload);
  IpComm::FpgaInfo config(fpga_info_payload);

  if(config.firmwareVersionMajor != LIBRARY_VERSION_MAJOR || config.firmwareVersionMinor != LIBRARY_VERSION_MINOR )
  {
    std::string error_msg =
        std::string("Sensor firmware version not compatible. Got ")
      + boost::lexical_cast<std::string>(config.firmwareVersionMajor)
      + std::string(".")
      + boost::lexical_cast<std::string>(config.firmwareVersionMinor)
      + std::string(".")
      + boost::lexical_cast<std::string>(config.firmwareVersionPatch)
      + std::string(" but expected ")
      + boost::lexical_cast<std::string>(LIBRARY_VERSION_MAJOR)
      + std::string(".")
      + boost::lexical_cast<std::string>(LIBRARY_VERSION_MINOR)
      + std::string(".X. Please use the visensor-update tool to update the firmware.\n");

    throw visensor::exceptions::FirmwareException(error_msg);
  }

  fpga_config_.fpga_id_ = config.fpgaId;
  fpga_config_.firmware_version_major = config.firmwareVersionMajor;
  fpga_config_.firmware_version_minor = config.firmwareVersionMinor;
  fpga_config_.firmware_version_patch = config.firmwareVersionPatch;
  fpga_config_.num_of_sensors_ = config.numOfSensors;

  VISENSOR_DEBUG("FPGA ID: fpgaId %d, Firmware Version: %d.%d.%d Data Streams: %d\n",
         fpga_config_.fpga_id_, fpga_config_.firmware_version_major,
         fpga_config_.firmware_version_minor,
         fpga_config_.firmware_version_patch, fpga_config_.num_of_sensors_);

  number_of_sensors_=fpga_config_.num_of_sensors_;
}

std::vector<IpComm::SensorInfo> IpConnection::getAttachedSensors()
{
  std::vector<IpComm::SensorInfo> sensor_list;

  sendHeader(IpComm::REQUEST_SENSOR_INFO);

  IpComm::Header header = readHeader(config_socket_);
  if(header.data_id != IpComm::SENSOR_INFO && header.data_size != number_of_sensors_)
  {
    VISENSOR_DEBUG("read init msg failed");
    return sensor_list;
  }

  if(number_of_sensors_==0)
    return sensor_list;

  for(int i=0;i<number_of_sensors_;++i)
  {
    // read sensor infos from socket
    IpComm::SensorInfoPayload sensor_info_payload;
    receive_payload(config_socket_, sensor_info_payload);

    IpComm::SensorInfo sensorInfo(sensor_info_payload);
    sensor_list.push_back(sensorInfo);
  }

  return sensor_list;
}

bool IpConnection::readCameraCalibration(SensorId::SensorId cam_id, unsigned int slot_id, ViCameraCalibration &calib_out)
{
  //request calibration with given id
  IpComm::Header header;
  header.data_id = IpComm::READ_CAMERA_CALIBRATION;
  header.data_size = 0;
  header.timestamp = 0;

  IpComm::CalibrationId package;
  package.port_id = cam_id;
  package.slot_id = slot_id;

  send_package(header, package);

  //PROCESS RESPONSE
  IpComm::Header header2;
  header2 = readHeader(config_socket_);

  if(header2.data_id != IpComm::READ_CAMERA_CALIBRATION)
    throw visensor::exceptions::FirmwareException("READ_CAMERA_CALIBRATION request failed. Check firmware version.");

  IpComm::CameraCalibrationPayload calib_payload;
  receive_payload(config_socket_, calib_payload);

  //deserialize
  IpComm::CameraCalibration calib(calib_payload);

  if(!calib.valid)
    return false;

  for(int i=0; i<2; i++) calib_out.focal_point[i] = calib.focal_point[i];
  for(int i=0; i<2; i++) calib_out.principal_point[i] = calib.principal_point[i];
  for(int i=0; i<5; i++) calib_out.dist_coeff[i] = calib.distortion[i];
  for(int i=0; i<9; i++) calib_out.R[i] = calib.R[i];
  for(int i=0; i<3; i++) calib_out.t[i] = calib.t[i];

  return true;
}

bool IpConnection::writeCameraCalibration(SensorId::SensorId cam_id, unsigned int slot_id, const ViCameraCalibration calib)
{
  //send calibration write request
  IpComm::Header header;
  header.data_id = IpComm::WRITE_CAMERA_CALIBRATION;
  header.data_size = 0;
  header.timestamp = 0;

  IpComm::CalibrationId package_id;
  package_id.port_id = cam_id;
  package_id.slot_id = slot_id;

  IpComm::CameraCalibration package_calib;
  for(int i=0; i<2; i++) package_calib.focal_point[i] = calib.focal_point[i];
  for(int i=0; i<2; i++) package_calib.principal_point[i] = calib.principal_point[i];
  for(int i=0; i<5; i++) package_calib.distortion[i] = calib.dist_coeff[i];
  for(int i=0; i<9; i++) package_calib.R[i] = calib.R[i];
  for(int i=0; i<3; i++) package_calib.t[i] = calib.t[i];

  boost::asio::write(config_socket_, boost::asio::buffer(header.getSerialized()));
  boost::asio::write(config_socket_, boost::asio::buffer(package_id.getSerialized()));
  boost::asio::write(config_socket_, boost::asio::buffer(package_calib.getSerialized()));

  //check if write was calibration was stored successfully (ACK/NACK)
  bool ret = receiveAck(config_socket_);
  return ret;
}

bool IpConnection::sendLedConfig(uint16_t value)
{
  return writeDataFpga(led_sensor_id_, 0x04, value);
}

// set lightning mode:
// 0 - strobe
// 1 - continuous
bool IpConnection::sendLedConfig(bool value)
{
  return writeDataFpga(led_sensor_id_, 0x00, value);
}

void IpConnection::sendExternalTriggerConfig(SensorId::SensorId sensor_id, const ViExternalTriggerConfig config)
{

  //from FpgaConstants.hpp (server file)
  const unsigned int R_EX_TRIGGER_CONTROL      = 0x00; // 0=OFF, 1=ON
  const unsigned int R_EX_TRIGGER_DIRECTION    = 0x04; // 0=out, 1=in
  const unsigned int R_EX_TRIGGER_POLARITY     = 0x08; // 0=active high, 1=active low
  const unsigned int R_EX_TRIGGER_LENGHT       = 0x0C; // impulse length in fpga zyklen (125MHz)


  //get trigger core status
  int core_running=1;
  readDataFpga(sensor_id, R_EX_TRIGGER_CONTROL, core_running);

  //turn core off, if running
  if(core_running != 0)
    writeDataFpga(sensor_id, R_EX_TRIGGER_CONTROL, 0);

  //set config
  writeDataFpga(sensor_id, R_EX_TRIGGER_POLARITY, (int)config.polarity);        //polarity
  writeDataFpga(sensor_id, R_EX_TRIGGER_DIRECTION, (int)config.direction);      //direction
  writeDataFpga(sensor_id, R_EX_TRIGGER_LENGHT, (int)FPGA_FREQUENCY/1000 * config.pulse_ms);  //pulse width (ms)

  //turn core back on, if it was running
  if(core_running != 0)
    writeDataFpga(sensor_id, R_EX_TRIGGER_CONTROL, 1);
}

void IpConnection::startSensor(SensorId::SensorId sensor_id, uint32_t rate)
{
  VISENSOR_DEBUG("starting sensor id %u rate %u\n",sensor_id,rate);
  sendStartSensor(sensor_id, rate);
}

void IpConnection::stopSensor(SensorId::SensorId sensor_id)
{
  VISENSOR_DEBUG("stop sensor id %u\n", sensor_id);
  sendStopSensor(sensor_id);
}

template <typename T>
bool IpConnection::writeDataUbi(SensorId::SensorId sens_addr, unsigned char reg,
                                 T val, int numBits) {
  IpComm::Header header;
  header.data_id = IpComm::WRITE_UBI_REGISTER;
  //VISENSOR_DEBUG("send header id: %d register: %d, val: %d\n", header.data_id, reg, val);
  IpComm::BusPackage package;
  package.sensor_id = sens_addr;
  package.NumBits = numBits;
  package.registerAddress = reg;
  package.value = val;

  send_package(header, package);

  return true;
}

bool IpConnection::writeDataFpga(SensorId::SensorId sens_addr, unsigned char reg,
                                 int val) {
  IpComm::Header header;
  header.data_id = IpComm::WRITE_FPGA_REGISTER;
  VISENSOR_DEBUG("write data fpga register: 0x%x, val: %u\n", reg, val);
  IpComm::BusPackage package;
  package.sensor_id = sens_addr;
  package.NumBits = 32;
  package.registerAddress = reg;
  package.value = val;

  send_package(header, package);

  return true;
}

template <typename T>
bool IpConnection::readDataUbi(SensorId::SensorId sens_addr, unsigned char reg,
                                 T *val, int numBits) {
  // formulate request
  IpComm::Header headerOut;
  headerOut.data_id = IpComm::READ_UBI_REGISTER;

  IpComm::BusPackage packageOut;
  packageOut.sensor_id = sens_addr;
  packageOut.NumBits = numBits;
  packageOut.registerAddress = reg;

  // send request
  send_package(headerOut, packageOut);

  // read answer
  IpComm::Header headerIn;
  IpComm::BusPackage packageIn;
  receive_package(headerIn, packageIn, IpComm::BusPackagePayload());

  //check data
  if(  headerIn.data_id != IpComm::READ_UBI_REGISTER)
    VISENSOR_DEBUG("readDataUbi: Wrong data received.\n");

  *val = packageIn.value;

  return true;
}

bool IpConnection::readDataFpga(SensorId::SensorId sens_addr, unsigned char reg,
                                 int &val) {
  // formulate request
  IpComm::Header headerOut;
  headerOut.data_id = IpComm::READ_FPGA_REGISTER;

  IpComm::BusPackage packageOut;
  packageOut.sensor_id = sens_addr;
  packageOut.NumBits = 32;
  packageOut.registerAddress = reg;

  // send request
  send_package(headerOut, packageOut);

  // read answer
  IpComm::Header headerIn;
  IpComm::BusPackage packageIn;
  receive_package(headerIn, packageIn, IpComm::BusPackagePayload());

  //check data
  if(  headerIn.data_id != IpComm::READ_FPGA_REGISTER)
    VISENSOR_DEBUG("readDataFpga: Wrong data received.\n");

  val = packageIn.value;

  return true;
}

void IpConnection::writeConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg, uint32_t val, uint8_t comType)
{
  if(comType==ViComType::I2C_16)
    writeDataUbi(sensor_id, reg, val, 16);//writeDataU16_i2c(sensors_.at(sensor_id)->stream_number_, dev_adress,reg,val);
  if(comType==ViComType::I2C_8)
    writeDataUbi(sensor_id, reg, val, 8);
  if(comType==ViComType::FPGA_32)
    writeDataFpga(sensor_id, reg, val);
}

void IpConnection::readConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg, uint32_t &val, uint8_t comType)
{
  if(comType==ViComType::I2C_16)
    readDataUbi(sensor_id, reg, &val, 16);
  if(comType==ViComType::I2C_8)
    readDataUbi(sensor_id, reg, &val, 8);
  // TODO(gohl): implement fpga read
//  if(comType==ViComType::FPGA_32)
//    readDataFpga(sensor_id, reg, &val);
}

bool IpConnection::readInitMsg(unsigned char * msg)
{
  //	unsigned char send_data[4] = {
  //			0x04 | 0x80, 0x00, 0x00, 0x00
  //	};
  //
  //	_fpgaEth.send(0,4,send_data);
  //	return getFPGAConfigStream(msg,16);
return true;
}

uint32_t IpConnection::getTimestampFpgaRaw(uint8_t* buffer)
{
  return (buffer[0]<<24) | (buffer[1]<<16) | (buffer[2]<<8) | (buffer[3]<<0);
}

uint64_t IpConnection::getTimestamp(uint8_t* buffer)
{
  //read timestamp  from buffer
  uint32_t packageTimestampRaw = getTimestampFpgaRaw(buffer);

  // FPGA timestamp is in 1/100000[s] -> change to nanoseconds
  uint64_t timestamp=time_synchronizer_.getSynchronizedTime((uint64_t)packageTimestampRaw*10000);

  return timestamp;
}

void IpConnection::processPackage(SensorId::SensorId sensor_id, Measurement::Ptr new_measurement)
{

//  VISENSOR_DEBUG("Payload: ");
//  for(int i = 0; i<nBytesReceived; i++)
//  {
//    if(i%2==0)
//      VISENSOR_DEBUG(" ");
//
//    VISENSOR_DEBUG("%02x",pkt_data[i]);
//  }
//  VISENSOR_DEBUG("\n");

  if(sensors_.count(sensor_id) == 0)
  {
    VISENSOR_DEBUG("no sensor initialized with sensor id %u\n", sensor_id);
    return;
  }

  // get current sensor object
  Sensor::Ptr sensor = sensors_.at(sensor_id);

  // don't process Sensor if not active
  if(!sensor->getSensorActive())
  {
    VISENSOR_DEBUG("sensor not active. id: %u\n", sensor_id);
    return;
  }

  // differ multi or single measurement packages
  if(sensor->getNumOfMsgsInPackage()==1)
  {
    if (sensor->getMeasurementBufferSize()
        > (int)new_measurement->buffer_size) {
      VISENSOR_DEBUG(
                  "sensor message smaller than expected. Got %u expected %u. sensor_id: %u\n",
                  new_measurement->buffer_size,
                  sensor->getMeasurementBufferSize()
                      * sensor->getNumOfMsgsInPackage(), sensor_id);
      return;
    }
    new_measurement->timestamp_host = time_synchronizer_.getSystemTime();
    sensor->addMeasurement(new_measurement);
  }
  else{
    // split the new measurement into single measurements
    for (int64_t i = 0; i < sensor->getNumOfMsgsInPackage(); i++) // TODO(schneith): make getNumOfMsgsInPackage dynamic
    {
      if (sensor->getMeasurementBufferSize() * (i) >= new_measurement->buffer_size) {
        VISENSOR_DEBUG(
            "imu message smaller than expected. Got %u expected %u.\n",
            new_measurement->buffer_size,
            sensor->getMeasurementBufferSize()
                * sensor->getNumOfMsgsInPackage());
        return;
      }

      uint8_t* data_ptr = new uint8_t[sensor->getMeasurementBufferSize()];
      memcpy(data_ptr, new_measurement->data .get()+ (sensor->getMeasurementBufferSize()*i),sensor->getMeasurementBufferSize());

      Measurement::Ptr single_measurement = boost::make_shared<Measurement>();
      single_measurement->data = boost::shared_ptr<uint8_t>( data_ptr, array_deleter<uint8_t>() );;
      single_measurement->buffer_size = sensor->getMeasurementBufferSize();
      single_measurement->timestamp = new_measurement->timestamp + i*sensor->getTimeBetweenMsgs();
      single_measurement->timestamp_host = time_synchronizer_.getSystemTime() + i*sensor->getTimeBetweenMsgs();

      //fpga timestamp
      uint64_t time_offset_fpga = (i*sensor->getTimeBetweenMsgs() * (uint64_t)FPGA_TIME_COUNTER_FREQUENCY)/(uint64_t)1e9;
      single_measurement->timestamp_fpga_counter = new_measurement->timestamp_fpga_counter + time_offset_fpga;

      sensor->addMeasurement(single_measurement);
    }
  }

}

}  //namespace visensor
