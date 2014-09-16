#ifndef IMU_ADIS16448_HPP_
#define IMU_ADIS16448_HPP_

#include <config/config.hpp>
#include <boost/smart_ptr.hpp>

#include "networking/connection.hpp"
#include "sensors/imu.hpp"

namespace visensor
{
namespace ImuAdis16448Defaults
{
const int
	RATE=IMU_FREQUENCY,
	MSG_SIZE=(10+6*2),
	NUM_OF_MSGS_IN_PACKAGE=10,
	CALIBRATION_SIZE=0;
const bool USE_CONST_PACKAGE_SIZE=true;
}

class ImuAdis16448 : public Imu {
public:
  typedef boost::shared_ptr<ImuAdis16448> Ptr;

	ImuAdis16448(SensorId::SensorId sensor_id, IpConnection::WeakPtr config_connection);
	virtual ~ImuAdis16448(){};

	virtual ViConfigMsg getConfigParam(std::string cmd, uint32_t value);
	//the threaded function that works on the queue of finished measurements
	void processMeasurements();
	virtual void init();
	static uint32_t calculateBufferSize();

private:
	void allocateBuffer();
	int getNumOfNewMsgs();
	void resetBuffer();
	void getGyro(uint8_t * buffer, double * gyro);
	void getAcc(uint8_t * buffer, double * acc);
	void getMag(uint8_t * buffer, double * mag);
	void getBaro(uint8_t * buffer, double * baro);
	void getTemp(uint8_t * buffer, double * temp);
	void newImuMsgs();
	uint64_t getTimestamp();
};
} //namespace visensor

#endif /* IMU_ADIS16448_HPP_ */
