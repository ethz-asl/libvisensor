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

#include "visensor/visensor_api.hpp"
#include "visensor_impl.hpp"

namespace visensor
{

ViSensorDriver::ViSensorDriver()
{
  pImpl_ = new Impl();
}

void ViSensorDriver::init()
{
	pImpl_->initAutodiscovery();
}

void ViSensorDriver::init(std::string hostname)
{
  pImpl_->init(hostname);
}

ViSensorDriver::~ViSensorDriver()//:_fpga(&getSensorFromID){
{
	delete pImpl_;
}

void ViSensorDriver::setCameraCallback(boost::function<void (ViFrame::Ptr)> callback)
{
  pImpl_->setCameraCallback(callback);
}

void ViSensorDriver::setCornerCallback(
    boost::function<void(ViCorner::Ptr)> callback) {
  pImpl_->setCornerCallback(callback);
}

void ViSensorDriver::setFramesCornersCallback(
    boost::function<
        void(ViFrame::Ptr, ViCorner::Ptr)> callback) {
  pImpl_->setFramesCornersCallback(callback);
}

void ViSensorDriver::setImuCallback(boost::function<void (boost::shared_ptr<ViImuMsg>)> callback)
{
  pImpl_->setImuCallback(callback);
}

void ViSensorDriver::startAllCameras(uint32_t rate) {
  pImpl_->startAllCameras(rate);
}

void ViSensorDriver::startSensor(SensorId::SensorId id, uint32_t rate)
{
  pImpl_->startSensor(id, rate);
}

void ViSensorDriver::stopSensor(SensorId::SensorId id)
{
  pImpl_->stopSensor(id);
}

void ViSensorDriver::setSensorConfigParam(SensorId::SensorId sensor_id,
                                                  std::string cmd,
                                                  int value) {
  pImpl_->setSensorConfigParam(sensor_id, cmd, value);
}

void ViSensorDriver::startAllCorners() {
  pImpl_->startAllCorners();
}

void ViSensorDriver::startAllImus(uint32_t rate) {
  pImpl_->startAllImus(rate);
}

std::vector<int> ViSensorDriver::getListOfCameraIDs() const
{
	return pImpl_->getListOfCameraIDs();
}

std::vector<int> ViSensorDriver::getListOfDenseIDs() const
{
  return pImpl_->getListOfDenseIDs();
}

std::vector<int> ViSensorDriver::getListOfCornerIDs() const {
  return pImpl_->getListOfCornerIDs();
}

std::vector<int> ViSensorDriver::getListOfImuIDs() const
{
	return pImpl_->getListOfImuIDs();
}

uint32_t ViSensorDriver::getFpgaId() const
{
	return pImpl_->getFpgaId();
}

void ViSensorDriver::downloadFile(std::string local_path, std::string remote_path) {
  pImpl_->downloadFile(local_path, remote_path);
}

void ViSensorDriver::uploadFile(std::string local_path, std::string remote_path) {
  pImpl_->uploadFile(local_path, remote_path);
}

bool ViSensorDriver::setLedConfigParam(std::string cmd, uint16_t value)
{
	return pImpl_->setLedConfigParam(cmd, value);
}

void ViSensorDriver::startAllExternalTriggers(uint32_t rate) {
  pImpl_->startAllExternalTriggers(rate);
}

void ViSensorDriver::setExternalTriggerCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  pImpl_->setExternalTriggerCallback(callback);
}

void ViSensorDriver::startAllDenseMatchers() {
  pImpl_->startAllDenseMatchers();
}

void ViSensorDriver::setDenseMatcherCallback(
    boost::function<void(ViFrame::Ptr)> callback) {
  pImpl_->setDenseMatcherCallback(callback);
}

bool ViSensorDriver::getCameraCalibration(unsigned int cam_id, ViCameraCalibration &calib, unsigned int slot_id) const
{
  return pImpl_->getCameraCalibration(cam_id, slot_id, calib);
}

bool ViSensorDriver::setCameraCalibration(unsigned int cam_id, const ViCameraCalibration calib, unsigned int slot_id ) const
{
  return pImpl_->setCameraCalibration(cam_id, slot_id, calib);
}

void ViSensorDriver::sendSerialData(ViSerialData::Ptr data) const
{
  //some data checks /TODO(schneith)

  pImpl_->sendSerialData(data);
}

void ViSensorDriver::setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback)
{
  pImpl_->setSerialCallback(callback);
}

bool ViSensorDriver::setSerialDelimiter(const char serial_id, const std::string delimiter) const
{
  return pImpl_->setSerialDelimiter(serial_id, delimiter);
}

bool ViSensorDriver::setSerialBaudrate(const char serial_id, const unsigned int baudrate) const
{
  return pImpl_->setSerialBaudrate(serial_id, baudrate);
}

}  //namespace visensor
