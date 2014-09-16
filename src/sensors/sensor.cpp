#include <boost/smart_ptr.hpp>

#include "networking/config_connection.hpp"
#include "sensors/sensor.hpp"

namespace visensor
{

Sensor::Sensor(SensorSettings sensor_settings, ConfigConnection::WeakPtr config_connection):
    config_connection_(config_connection),
    settings_(sensor_settings),
    prev_timestamp_(0),
    allowed_timediff_(0),
    measurement_miss_counter_(0)
{
}

Sensor::~Sensor()
{
}

//lock weak pointer for usage
ConfigConnection::Ptr Sensor::getConfigConnection() {
  return config_connection_.lock();
}

void Sensor::writeRequest(const ViConfigMsg configMsg)
{
  getConfigConnection()->writeConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg,configMsg.val, configMsg.comType);
  return;
}

void Sensor::readRequest(ViConfigMsg& configMsg)
{
  getConfigConnection()->readConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg, configMsg.val, configMsg.comType);
  return;
}

bool Sensor::checkTimestamp(uint64_t timestamp)
{
  bool frameMissed = false;

  //init with first timestamp
	if(prev_timestamp_==0)
	{
		prev_timestamp_=timestamp;
		return frameMissed;
	}

	if(timestamp-prev_timestamp_>allowed_timediff_)
	{
	  measurement_miss_counter_++;
		VISENSOR_DEBUG("sensor %d frame missed (total misses: %u), timediff: %ld ns\n", id(), measurement_miss_counter_, timestamp-prev_timestamp_);
		frameMissed = true;
	}

	prev_timestamp_=timestamp;
	return frameMissed;
}

int Sensor::getMeasurementBufferSize() const
{
  return settings_.measurementBufferSize;
}

void Sensor::addMeasurement(const Measurement::Ptr meas){
  measurement_queue_.push(meas);
}

//return time between msgs in nanoseconds
uint64_t Sensor::getTimeBetweenMsgs() const
{
	// avoid division by 0, just return 0 if rate is not set yet
	if(settings_.rate==0)
		return 0;

	return 1000000000/settings_.rate;
}

int Sensor::getNumOfMsgsInPackage() const
{
	return settings_.numOfMsgsInPackage;
}

// start sensor with given rate
bool Sensor::startSensor(uint32_t rate)
{
	settings_.rate=rate;
	settings_.active=true;
	allowed_timediff_=1.5*getTimeBetweenMsgs();
	return true;
}

// stop sensor
bool Sensor::stopSensor()
{
	settings_.active=false;
	return true;
}

}//namespace visensor
