/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Redistribution and non-commercial use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 *  Neither the name of the {organization} nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
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

#ifndef VISENSOR_CONSTANTS_HPP_
#define VISENSOR_CONSTANTS_HPP_

#include <visensor/visensor_config.hpp>
#include <stdint.h>

namespace visensor {
  #define CAMERA_FREQUENCY 20 // in Hz
  #define IMU_FREQUENCY 200 // in Hz
  #define FPGA_FREQUENCY 125000000
  #define FPGA_TIME_COUNTER_FREQUENCY 100000.0 //fpga timestamp counter frequency

  namespace ViComType {
    static const int SPI_8 = 0, SPI_16 = 1, SPI_32 = 2, I2C_8 = 3, I2C_16 = 4, I2C_32 = 5, FPGA_32 = 6;
  }

  namespace SensorType {
    enum SensorType {
      CAMERA_MT9V034 = 1,
      IMU_ADIS16488 = 2,
      IMU_MPU6000 = 3,		// mpu of prototype 0
      IMU_ADIS16375 = 4,
      IMU_MPU6000_P1 = 5,  // mpu of prototype 1
      CAMERA_TAU320 = 6,
      CORNER_MT9V034 = 7,
      IMU_ADXRS450 = 8,
      BRISK_MT9V034 = 9,
      TIMING_BLOCK = 10,
      IMU_ADIS16448 = 11,
      CAMERA_TAU640 = 12,
      LIGHT_CONTROL = 13,
      ZYNQ_STATUS = 14,
      DENSE_MATCHER = 15,
      EXTERNAL_TRIGGER = 16,
      MPU_9150 = 17
    };
  }  //namespace SensorType

  namespace SensorId {
    enum SensorId {
      CAM0 = 0,
      CAM1 = 1,
      CAM2 = 2,
      CAM3 = 3,
      IMU0 = 4,
      IMU_CAM0 = 5,
      IMU_CAM1 = 6,
      IMU_CAM2 = 7,
      IMU_CAM3 = 8,
      CORNER_CAM0 = 9,
      CORNER_CAM1 = 10,
      CORNER_CAM2 = 11,
      CORNER_CAM3 = 12,
      DENSE_MATCHER0 = 13,
      EXTERNAL_TRIGGER0 = 14,
      SENSOR_STATUS = 15,
      SENSOR_CLOCK = 16,
      FLIR0 = 17,
      FLIR1 = 18,
      FLIR2 = 19,
      FLIR3 = 20,
      LED_FLASHER0 = 21,
      NOT_SPECIFIED
    };

    DSO_EXPORT inline SensorId getCamId(SensorId id) {
      switch (id) {
        case (CORNER_CAM0):
        case (DENSE_MATCHER0):
          return CAM0;
        case (CORNER_CAM1):
          return CAM1;
        case (CORNER_CAM2):
          return CAM2;
        case (CORNER_CAM3):
          return CAM3;
        default:
          return NOT_SPECIFIED;
      }
    }
  }  //namespace SensorPort

}  //namespace visensor

#endif /* VISENSOR_CONSTANTS_HPP_ */
