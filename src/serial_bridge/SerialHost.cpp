/*
 * SerialHost.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: schneith
 */

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

#include "visensor/visensor_datatypes.hpp"
#include "serial_bridge/SerialHost.hpp"

namespace visensor {

SerialHost::SerialHost()
{
  //start processing thread
  worker_thread = boost::make_shared<boost::thread>(&SerialHost::processSerialData, this);
}

SerialHost::~SerialHost() {
  //stop thread
  try {
    if(worker_thread)
      worker_thread->interrupt();
  } catch (const std::exception &ex) {
    VISENSOR_DEBUG("SerialHost exception in destructor: %s\n", ex.what());
  }

  //wait for it to stop
  worker_thread->join();
}

//the threaded function that works on the queue of finished measurements
void SerialHost::processSerialData() {

  while (1) {
    boost::this_thread::interruption_point();

    //get the newest measurement (waits if no new msgs available..)
    ViSerialData::Ptr data_ptr = data_queue_.pop();

    //send out to user application
    publishSerialData(data_ptr);
  }
}

void SerialHost::setSerialDataCallback(boost::function<void(ViSerialData::Ptr)> callback) {
  user_callbacks_.push_back(callback);
}

void SerialHost::publishSerialData(ViSerialData::Ptr& data_ptr) {
  if (user_callbacks_.empty() == 0) {
    BOOST_FOREACH( boost::function<void(ViSerialData::Ptr)> callback, user_callbacks_)
        { callback(data_ptr); }
  }
}

void SerialHost::addDataToPublishQueue(ViSerialData::Ptr data_ptr)
{
  data_queue_.push(data_ptr);
}

} /* namespace visensor */
