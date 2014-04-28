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
 *     http://www.apache.org/licenses/LICENSE-2.0
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

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "visensor/visensor_exceptions.hpp"
#include "networking/auto_discovery.hpp"
#include "sensors/sensor_factory.hpp"
#include "serial_bridge/SerialHost.hpp"

#include "visensor_impl.hpp"

namespace visensor {

ViSensorDriver::Impl::Impl()  //:_fpga(&getSensorFromID){
{
  ip_connection_ = boost::make_shared<IpConnection>();
}

void ViSensorDriver::Impl::initAutodiscovery()
{
  //find address using AutoDiscovery
  std::string sensor_address;

  boost::asio::io_service io_service_;
  AutoDiscovery sensor_finder(13779);
  sensor_address = sensor_finder.findSensor();

  if (sensor_address.compare("0.0.0.0"))
    VISENSOR_DEBUG("Autodiscovery: found sensor with IP %s\n", sensor_address.c_str());
  else {
    VISENSOR_DEBUG("Autodiscovery: could not find a sensor.\n");
    throw visensor::exceptions::ConnectionException("Autodiscovery: could not find a sensor.");
  }

  //init with found IP address
  init(sensor_address);
}

void ViSensorDriver::Impl::init(std::string hostname) {
  // create fpga interface
  ip_connection_->connect(hostname);

  // get a list of all sensors connected to the fpga
  std::vector<IpComm::SensorInfo> sensor_list = ip_connection_->getAttachedSensors();

  // create all the sensor objects
  SensorFactory::createSensors(ip_connection_, &sensors_, &sensor_threads_);

  for (Sensor::IdMap::iterator it = sensors_.begin(); it != sensors_.end();
      ++it) {
    // inform comm about sensor
    ip_connection_->addSensor(it->second->stream_id(), it->second);
    // init sensor
    it->second->init();
  }

  //initialize the serial bridge
  serial_host_ = boost::make_shared<SerialHost>();
  ip_connection_->registerSerialHost(serial_host_);

  //set initialized flag
  initialized_ = true;
}

ViSensorDriver::Impl::~Impl()  //:_fpga(&getSensorFromID){
{
  sensor_threads_.interrupt_all();
  sensor_threads_.join_all();

  // stop all sensors
  for (Sensor::IdMap::iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    ip_connection_->stopSensor(it->second->stream_id());
  }
}

bool ViSensorDriver::Impl::isInitialized(void)
{
  return initialized_;
}

void ViSensorDriver::Impl::startAllCameras(uint32_t rate) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CAMERA_MT9V034)
      startSensor(it->first, rate);
  }
}

void ViSensorDriver::Impl::setCameraCallback(
    boost::function<void(ViFrame::Ptr)> callback) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CAMERA_MT9V034)
      it->second->setFrameCallback(callback);
  }
}

void ViSensorDriver::Impl::setImuCallback(
    boost::function<void(boost::shared_ptr<ViImuMsg>)> callback) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::IMU_ADIS16448)
      it->second->setUserCallback(callback);
  }
}

void ViSensorDriver::Impl::startSensor(SensorId::SensorId sensor_id, uint32_t rate) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("startSensor: Invalid sensor id" + sensor_id);

  //TODO(gohlp) move rate check to sensor class
  if (rate == 0 || rate > 1000)
    throw visensor::exceptions::SensorException("startSensor: Invalid rate for sensor " + sensor_id);

  // set sensor active
  sensors_.at(sensor_id)->startSensor(rate);

  // start triggering the sensor on the fpga
  ip_connection_->startSensor(sensors_.at(sensor_id)->stream_id(), rate);
}

void ViSensorDriver::Impl::stopSensor(SensorId::SensorId sensor_id) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("stopSensor: Invalid sensor id " + sensor_id);

  // stop triggering the sensor on the fpga
  ip_connection_->stopSensor(sensors_.at(sensor_id)->stream_id());

  sensors_.at(sensor_id)->stopSensor();
}

void ViSensorDriver::Impl::startAllImus(uint32_t rate) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::IMU_ADIS16448)
          startSensor(it->first, rate);
  }
}

std::vector<int> ViSensorDriver::Impl::getListOfCameraIDs() const {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<int> list_of_cameras;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::CAMERA_MT9V034)
    list_of_cameras.push_back(it->first);
  }
  return list_of_cameras;
}

std::vector<int> ViSensorDriver::Impl::getListOfDenseIDs() const {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<int> list_of_dense;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::DENSE_MATCHER)
    list_of_dense.push_back(it->first);
  }
  return list_of_dense;
}

std::vector<int> ViSensorDriver::Impl::getListOfImuIDs() const {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<int> list_of_imus;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::IMU_ADIS16448)
    list_of_imus.push_back(it->first);
  }
  return list_of_imus;
}

std::vector<int> ViSensorDriver::Impl::getListOfCornerIDs() const {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  std::vector<int> list_of_corners;
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::CORNER_MT9V034)
    list_of_corners.push_back(it->first);
  }
  return list_of_corners;
}

uint32_t ViSensorDriver::Impl::getFpgaId() const {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  return ip_connection_->getId();
}

void ViSensorDriver::Impl::setSensorConfigParam(SensorId::SensorId sensor_id,
                                                  std::string cmd,
                                                  uint16_t value) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  // check if id is valid
  if (sensors_.count(sensor_id) == 0)
    throw visensor::exceptions::SensorException("setSensorConfigParam: Invalid sensor id:  " + sensor_id);

  ViConfigMsg msg = sensors_.at(sensor_id)->getConfigParam(cmd, value);

  if (msg.valChanged) {
    VISENSOR_DEBUG(
        "configRequest \"%s\" : id %#X, addr %#X, reg %#X,val %#X type %#X \n",
        cmd.c_str(), msg.sensorId, msg.devAdress, msg.reg, msg.val,
        msg.comType);
    ip_connection_->writeConfig(msg.sensorId, msg.devAdress, msg.reg, msg.val,
                                msg.comType);
  }
}

void ViSensorDriver::Impl::downloadFile(std::string& local_path,
                                          std::string& remote_path) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->downloadFile(local_path, remote_path);
}

void ViSensorDriver::Impl::startAllExternalTriggers(uint32_t rate) {
  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
      startSensor(it->first, rate);
  }
}

void ViSensorDriver::Impl::setExternalTriggerCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
        it != sensors_.end(); ++it) {
      if (it->second->type() == SensorType::EXTERNAL_TRIGGER)
    it->second->setUserCallback(callback);
  }
}

void ViSensorDriver::Impl::startAllDenseMatchers() {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::DENSE_MATCHER)
      startSensor(it->first, 1);
  }
}

void ViSensorDriver::Impl::setDenseMatcherCallback(
    boost::function<void(ViFrame::Ptr)> callback) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::DENSE_MATCHER)
      it->second->setFrameCallback(callback);
  }
}

void ViSensorDriver::Impl::sendSerialData(ViSerialData::Ptr data)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  //send the data
  ip_connection_->sendSerialData(data);
}

void ViSensorDriver::Impl::setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  //register callback
  serial_host_->setSerialDataCallback(callback);
}

bool ViSensorDriver::Impl::setSerialDelimiter(const char serial_id, const std::string delimiter)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  return ip_connection_->setSerialDelimiter(serial_id, delimiter);
}

bool ViSensorDriver::Impl::setSerialBaudrate(const char serial_id, const unsigned int baudrate)
{
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  return ip_connection_->setSerialBaudrate(serial_id, baudrate);
}

void ViSensorDriver::Impl::uploadFile(std::string& local_path,
                                        std::string& remote_path) {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  ip_connection_->uploadFile(local_path, remote_path);
}

bool ViSensorDriver::Impl::setLedConfigParam(std::string cmd,
                                               uint16_t value) {
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  //TODO(gohlp) make a LED map to access only LEDs
//  return ip_connection_->sendLedConfig((bool) (value & 0x0001));
  return 0;
}

void ViSensorDriver::Impl::setCornerCallback(
    boost::function<void(ViCorner::Ptr)> callback) {

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034)
      it->second->setCornerCallback(callback);
  }
}

void ViSensorDriver::Impl::setFramesCornersCallback(
    boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback) {

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034) {

      // check if corresponding camera is available
      const SensorId::SensorId& corner_id = it->first;
      const SensorId::SensorId& camera_id = SensorId::getCamId(corner_id);

      if (sensors_.count(camera_id) == 0) {
        VISENSOR_DEBUG("Camera %d not available to sync corners %d with.\n", camera_id,
               it->first);
        continue;
      }

      VISENSOR_DEBUG("start camera corner synchronizer for cam %d", corner_id);

      cam_corner_syncronizer_.push_back(new FrameCornerSynchronizer());

      // register all the callbacks
      cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1]
          ->setUserCallback(callback);
      sensors_.at(corner_id)->setCornerCallback(
          boost::bind(
              &FrameCornerSynchronizer::addCorner,
              cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1], _1));
      sensors_.at(camera_id)->setFrameCallback(
          boost::bind(
              &FrameCornerSynchronizer::addFrame,
              cam_corner_syncronizer_[cam_corner_syncronizer_.size() - 1], _1));
  }
  }
}

void ViSensorDriver::Impl::startAllCorners() {

  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");

  for (Sensor::IdMap::const_iterator it = sensors_.begin();
      it != sensors_.end(); ++it) {
    if (it->second->type() == SensorType::CORNER_MT9V034)
      startSensor(it->first, 1);
  }
}

bool ViSensorDriver::Impl::getCameraCalibration(unsigned int cam_id, unsigned int slot_id, ViCameraCalibration &calib){
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  return ip_connection_->readCameraCalibration(cam_id, slot_id, calib);
}

bool ViSensorDriver::Impl::setCameraCalibration(unsigned int cam_id, unsigned int slot_id, const ViCameraCalibration calib){
  if(!initialized_) throw visensor::exceptions::ConnectionException("No connection to sensor.");
  //slot 0 holds the factory calibration and can't be overwritten using the public API
  if(slot_id == 0)
    return false;

  return ip_connection_->writeCameraCalibration(cam_id, slot_id, calib);
}


}  //namespace visensor
