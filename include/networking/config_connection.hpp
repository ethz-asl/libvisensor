/*
 * ConfigConnection.hpp
 *
 *  Created on: Dec 4, 2013
 *      Author: pascal
 */

#ifndef CONFIGCONNECTION_HPP_
#define CONFIGCONNECTION_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include <visensor/visensor_constants.hpp>

namespace visensor {

class ConfigConnection {
 public:

  typedef boost::shared_ptr<ConfigConnection> Ptr;
  typedef boost::weak_ptr<ConfigConnection> WeakPtr;

  ConfigConnection() {};
  virtual ~ConfigConnection() {};

  virtual void writeConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                           uint32_t val, uint8_t comType) = 0;
  virtual void readConfig(SensorId::SensorId sensor_id, uint8_t dev_adress, uint8_t reg,
                          uint32_t &val, uint8_t comType) = 0;
};

} /* namespace visensor */

#endif /* CONFIGCONNECTION_HPP_ */
