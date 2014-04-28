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

#ifndef VISENSOR_API_HPP_
#define VISENSOR_API_HPP_

#include <visensor/visensor_config.hpp>

#include <string>
#include <vector>
#include <stdint.h>
#include <boost/function.hpp>

#include <visensor/visensor_datatypes.hpp>
#include <visensor/visensor_constants.hpp>

namespace visensor {

class DSO_EXPORT ViSensorDriver {

 public:
  typedef boost::shared_ptr<ViSensorDriver> Ptr;

  ViSensorDriver();
  ~ViSensorDriver();

  void init(); //init using autodiscovery
  void init(std::string host_ip);

  void startSensor(visensor::SensorId::SensorId sensor_id, uint32_t rate =
                       CAMERA_FREQUENCY);
  void stopSensor(visensor::SensorId::SensorId sensor_id);
  void setSensorConfigParam(visensor::SensorId::SensorId sensor_id, const std::string cmd,
                            int value);

  void startAllCameras(uint32_t rate = CAMERA_FREQUENCY);
  void setCameraCallback(boost::function<void(ViFrame::Ptr)> callback);

  void startAllCorners();
  void setCornerCallback(boost::function<void(ViCorner::Ptr)> callback);

  void startAllImus(uint32_t rate = IMU_FREQUENCY);
  void setImuCallback(boost::function<void(ViImuMsg::Ptr)> callback);

  void startAllExternalTriggers(uint32_t rate);
  void setExternalTriggerCallback(
      boost::function<void(ViExternalTriggerMsg::Ptr)> callback);

  void startAllDenseMatchers();
  void setDenseMatcherCallback(
      boost::function<void(ViFrame::Ptr)> callback);

  // is called with synchronized images and corresponding corners
  void setFramesCornersCallback(
      boost::function<void(ViFrame::Ptr, ViCorner::Ptr)> callback);

  std::vector<int> getListOfCameraIDs() const;
  std::vector<int> getListOfDenseIDs() const;
  std::vector<int> getListOfCornerIDs() const;
  std::vector<int> getListOfImuIDs() const;
  uint32_t getFpgaId() const;

  //slot 0 holds the factory calibration (and can't be overwritten using the public API)
  bool getCameraCalibration(unsigned int cam_id, ViCameraCalibration& calib, unsigned int slot_id=0) const;
  bool setCameraCalibration(unsigned int cam_id, const ViCameraCalibration calib, unsigned int slot_id=1) const;

  //serial port bridge
  void sendSerialData(ViSerialData::Ptr data) const;
  void setSerialCallback(boost::function<void(ViSerialData::Ptr)> callback);
  bool setSerialDelimiter(const char serial_id, const std::string delimiter) const;
  bool setSerialBaudrate(const char serial_id, const unsigned int baudrate) const;

  //led configuration
  bool setLedConfigParam(const std::string cmd, uint16_t value);

  //file transfer
  void downloadFile(std::string local_path, std::string remote_path);
  void uploadFile(std::string local_path, std::string remote_path);

 private:
  class Impl;
  Impl* pImpl_;

};
}  //namespace visensor

#endif /* VISENSOR_API_HPP_ */
