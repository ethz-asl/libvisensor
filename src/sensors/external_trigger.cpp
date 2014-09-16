#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>

#include "networking/connection.hpp"
#include "sensors/external_trigger.hpp"

namespace visensor {

ExternalTrigger::ExternalTrigger(SensorId::SensorId sensor_id,
                                 IpConnection::WeakPtr config_connection)
    : Sensor(
          SensorSettings(sensor_id,
                         SensorType::SensorType::EXTERNAL_TRIGGER,
                         calculateBufferSize(),
                         ExternalTriggerDefaults::NUM_OF_MSGS_IN_PACKAGE, 0, 0),
          config_connection),
          config_(sensor_id) {
}

void ExternalTrigger::setUserCallback(
    boost::function<void(ViExternalTriggerMsg::Ptr)> callback) {
  user_callback_ = callback;
}

ViConfigMsg ExternalTrigger::getConfigParam(std::string cmd, uint32_t value) {
  return config_.getConfigParam(cmd, value);
}

void ExternalTrigger::publishExternalTriggerData(
    ViExternalTriggerMsg::Ptr msg) {
  if (user_callback_)
    user_callback_(msg);
}

uint32_t ExternalTrigger::calculateBufferSize() {
  return (ExternalTriggerDefaults::MSG_SIZE);
}

void ExternalTrigger::init() {
}

void ExternalTrigger::processMeasurements() {
  while (1) {

    boost::this_thread::interruption_point();
    //get the newest measurement
    Measurement::Ptr meas = Sensor::measurement_queue_.pop();

    // create new shared pointer for the message
    ViExternalTriggerMsg::Ptr trigger_msg_ptr = boost::make_shared<ViExternalTriggerMsg>();

    uint8_t* buffer = meas->data.get();
    uint8_t id = static_cast<int8_t>(buffer[0]);

    trigger_msg_ptr->trigger_id = id;
    trigger_msg_ptr->timestamp = meas->timestamp;
    trigger_msg_ptr->timestamp_host = meas->timestamp_host;
    trigger_msg_ptr->timestamp_fpga_counter = meas->timestamp_fpga_counter;
    trigger_msg_ptr->event_id = *(uint32_t*)(meas->data.get() + 1); //counts all trigger events (allows for event missed checking)
    publishExternalTriggerData(trigger_msg_ptr);

  }
}
}  //namespace visensor
