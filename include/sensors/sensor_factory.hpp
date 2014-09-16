/*
 * sensor_factory.hpp
 *
 *  Created on: Apr 17, 2012
 *      Author: burrimi
 */

#ifndef SENSOR_FACTORY_HPP_
#define SENSOR_FACTORY_HPP_

#include <config/config.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread.hpp>

#include "networking/connection.hpp"
#include "sensors/sensor.hpp"

namespace visensor {
class SensorFactory {
 public:
  static const void createSensors(
      IpConnection::WeakPtr config_connection,
      Sensor::IdMap* sensor_map,
      boost::thread_group* threads);
};

}  //namespace visensor

#endif /* SENSOR_FACTORY_HPP_ */
