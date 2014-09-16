#ifndef IMU_ADIS16488_HPP_
#define IMU_ADIS16488_HPP_

#include "imu.hpp"

namespace visensor
{

namespace ImuAdis16488Defaults
{
const int
	RATE=IMU_FREQUENCY,
	MSG_SIZE=(6*4+3*2+4),
	NUM_OF_MSGS_IN_PACKAGE=50,
	CALIBRATION_SIZE=0;
const bool USE_CONST_PACKAGE_SIZE=true;
}

class ImuAdis16488 : public Imu {
public:
	ImuAdis16488(SensorId::SensorId sensor_id, int stream_id, IpConnection::WeakPtr config_connection);
	virtual ~ImuAdis16488(){};

	virtual ViConfigMsg getConfigParam(std::string cmd, uint16_t value);
	//the threaded function that works on the queue of finished measurements
	void processMeasurements();
	virtual bool init();
	virtual bool setCalibration(CalibBase::Ptr calibration);
	static uint32_t calculateBufferSize();

private:
	void allocateBuffer();
	int getNumOfNewMsgs();
	void resetBuffer();
	void getGyro(uint8_t * buffer, double * gyro);
	void getAcc(uint8_t * buffer, double * acc);
	void getMag(uint8_t * buffer, double * mag);
	void getBaro(uint8_t * buffer, double * baro);
	void newImuMsgs();
	uint64_t getTimestamp();
};
} //namespace visensor

#endif /* IMU_ADIS16488_HPP_ */
