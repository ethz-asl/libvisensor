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

#ifndef VISENSOR_DATATYPES_HPP_
#define VISENSOR_DATATYPES_HPP_

#include <stdint.h> // for uint32_t definitions
#include <vector>

#include <boost/shared_ptr.hpp>

#include <visensor/visensor_config.hpp>
#include <visensor/visensor_constants.hpp>

namespace visensor {

  #define BRISK_DESCRIPTOR_SIZE 48

  typedef enum {
    NOT_SET = 0,
    MONO8 = 2,
    COLOR = 4,
    MONO16 = 5
  } ViImageType;

  struct DSO_EXPORT ViConfigMsg {
    SensorId::SensorId sensorId; /* id of the sensor */
    uint8_t devAdress; /* i2c address of the device */
    uint32_t reg; /* register to change */
    uint32_t val; /* new register value */
    uint8_t comType; /* type of communication to the sensor */
    bool valChanged; /* true, if value changed */

    typedef boost::shared_ptr<ViConfigMsg> Ptr;
  };

  class DSO_EXPORT ViConfigParam {

   public:
    ViConfigParam()
        : _register(0),
          _default(0),
          _min(0),
          _max(0),
          _scaling(0),
          _value(0) {};

    ViConfigParam(uint8_t reg, uint16_t def, uint16_t min, uint16_t max,
                      float scaling)
        : _register(reg),
          _default(def),
          _min(min),
          _max(max),
          _scaling(scaling),
          _value(0) {};

   public:
    uint8_t _register;
    uint16_t _default;
    uint16_t _min;
    uint16_t _max;
    float _scaling;
    float _value;

    typedef boost::shared_ptr<ViConfigParam> Ptr;
  };

  struct DSO_EXPORT ViImuConfig {
    uint32_t rate; /* the imu data rate */

    typedef boost::shared_ptr<ViImuConfig> Ptr;
  };

  struct DSO_EXPORT ViExternalTriggerConfig {
    enum direction_e {
      TRIGGER_OUTPUT=0,
      TRIGGER_INPUT=1
    };

    enum polarity_e {
      TRIGGER_ACTIVE_HIGH=0,
      TRIGGER_ACTIVE_LOW=1
    };

    direction_e direction;
    polarity_e polarity;

    unsigned int pulse_ms; //ms

    typedef boost::shared_ptr<ViExternalTriggerConfig> Ptr;
  };

  struct DSO_EXPORT ViImuMsg {
    double gyro[3]; /* 3D Gyro values (rad/s) */
    double acc[3]; /* 3D acceleration values (m^2/s) */
    double mag[3]; /* 3D magnetometer values (gauss) */
    double baro; /* barometer value (hPa) */
    double temperature; /* temperature value */
    uint64_t timestamp; /* time of imu message (fpga time)*/
    uint64_t timestamp_host; /* time when imu message was received on host*/
    uint32_t timestamp_fpga_counter; /* raw fpga counter in "FPGA_TIME_COUNTER_FREQUENCY" Hz */
    uint32_t imu_id; /* id of the imu */

    typedef boost::shared_ptr<ViImuMsg> Ptr;
  };

  struct DSO_EXPORT ViExternalTriggerMsg {
    uint64_t timestamp; /* time of trigger message (fpga time)*/
    uint64_t timestamp_host; /* time when imu message was received on host*/
    uint32_t timestamp_fpga_counter; /* raw fpga counter in "FPGA_TIME_COUNTER_FREQUENCY" Hz */

    uint32_t trigger_id; /* id of the trigger core */
    uint32_t event_id; /* id of the trigger event */

    typedef boost::shared_ptr<ViExternalTriggerMsg> Ptr;
  };

  struct DSO_EXPORT ViCornerElem {
    ViCornerElem()
        : x(0),
          y(0),
          score(0),
          descriptor_size(BRISK_DESCRIPTOR_SIZE) {
    };

    uint16_t x;
    uint16_t y;
    uint32_t score;
    uint8_t descriptor_size;
    uint8_t descriptor[BRISK_DESCRIPTOR_SIZE];

    typedef boost::shared_ptr<ViCornerElem> Ptr;
  };

  struct DSO_EXPORT ViCorner {
    std::vector<ViCornerElem> corners;
    uint64_t timestamp; /* time when the image was captured*/
    uint64_t timestamp_host; /* time when the data was received on host computer*/
    uint32_t timestamp_fpga_counter; /* raw fpga counter in "FPGA_TIME_COUNTER_FREQUENCY" Hz */
    uint32_t camera_id; /* the id of the camera */

    typedef boost::shared_ptr<ViCorner> Ptr;
  };

  class DSO_EXPORT ViFrame {
   public:
    ViFrame();
    ~ViFrame();

    int getBufferSize();
    void setImageRawPtr(boost::shared_ptr<uint8_t> buffer, uint32_t buffer_size);
    uint8_t* getImageRawPtr();

    uint32_t width; /* the image width */
    uint32_t height; /* the image height */
    uint64_t timestamp; /* time when the image was captured (fpga time)*/
    uint64_t timestamp_host; /* time when the image was received on host computer*/
    uint32_t timestamp_fpga_counter; /* raw fpga counter in "FPGA_TIME_COUNTER_FREQUENCY" Hz */
    uint32_t id; /* the frame id */
    uint32_t allocated_image_bytes; /* amount of memory allocated for the *image field. */
    uint32_t camera_id; /* the id of the camera */
    ViImageType image_type; /* type of the image */
    bool useCorners; /* */
    ViCorner::Ptr corner;

    typedef boost::shared_ptr<ViFrame> Ptr;

   private:
    boost::shared_ptr<uint8_t> image; /* the image */
  };

  struct DSO_EXPORT ViCameraCalibration {
    //intrinsics
    double focal_point[2];
    double principal_point[2];
    double dist_coeff[5];

    //extrinsics
    double R[9];
    double t[3];

    typedef boost::shared_ptr<ViCameraCalibration> Ptr;
  };

  struct DSO_EXPORT ViSerialData {
    enum direction_e {
      FROM_SERIAL_DEVICE,
      TO_SERIAL_DEVICE
    };

    unsigned int port_id;
    direction_e direction;
    std::string data;

    typedef boost::shared_ptr<ViSerialData> Ptr;
  };
  typedef std::vector<std::string> ViDeviceList;

  //ViSensor error codes
  enum class ViErrorCodes{
    NO_ERROR,
    MEASUREMENT_DROPPED
  };
  typedef const ViErrorCodes& ViErrorCode;

}  //namespace visensor

#endif /* VISENSOR_DATATYPES_HPP_ */
