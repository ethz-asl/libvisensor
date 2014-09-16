/*
 * SerialHost.hpp
 *
 *  Created on: Mar 5, 2014
 *      Author: schneith
 */

#ifndef SERIALHOST_HPP_
#define SERIALHOST_HPP_

#include <config/config.hpp>

#include <queue>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include "synchronization/concurrent_queue.hpp"
#include "visensor/visensor_datatypes.hpp"

namespace visensor {

class SerialHost {

 private:
  void processSerialData();
  void publishSerialData(ViSerialData::Ptr& data_ptr);

  boost::shared_ptr<boost::thread> worker_thread;

  std::vector<boost::function<void(ViSerialData::Ptr)> > user_callbacks_;
  concurrent_queue<std::queue<ViSerialData::Ptr> > data_queue_;

 public:
  SerialHost();
  virtual ~SerialHost();

  void setSerialDataCallback(boost::function<void(ViSerialData::Ptr)> callback);
  void addDataToPublishQueue(ViSerialData::Ptr data_ptr);

  typedef boost::shared_ptr<SerialHost> Ptr;
  typedef boost::weak_ptr<SerialHost> WeakPtr;
};

} /* namespace visensor */
#endif /* SERIALHOST_HPP_ */
