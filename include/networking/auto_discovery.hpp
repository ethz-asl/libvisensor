/*
 * AutoDiscovery.hpp
 *
 *  Created on: Aug 19, 2013
 *      Author: pascal
 */

#ifndef AutoDiscovery_HPP_
#define AutoDiscovery_HPP_

#include <config/config.hpp>
#include <set>
#include <vector>
#include <boost/asio.hpp>

#include "visensor/visensor_datatypes.hpp"

namespace visensor
{

class AutoDiscovery {
 public:

  AutoDiscovery(unsigned short int port);
  virtual ~AutoDiscovery();

  ViDeviceList findSensor();

 private:
  typedef std::set<std::string> SensorSet;
  std::vector<boost::asio::ip::address_v4> getIpList();

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  unsigned short int port_;
  char* data_;
  boost::asio::ip::udp::endpoint sensor_endpoint_;
};

} //namespace visensor

#endif /* AutoDiscovery_HPP_ */
