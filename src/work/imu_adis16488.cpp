#include <imu_adis16488.hpp>

namespace visensor
{


ImuAdis16488::ImuAdis16488(SensorId::SensorId sensor_id, int stream_id, IpConnection::WeakPtr config_connection):
		Imu(imu_id,
				SensorSettings(sensor_id,SensorType::SensorType::IMU_ADIS16488,calculateBufferSize(),
						ImuAdis16488Defaults::NUM_OF_MSGS_IN_PACKAGE,
						ImuAdis16488Defaults::USE_CONST_PACKAGE_SIZE,
						ImuAdis16488Defaults::CALIBRATION_SIZE),
						config_connection)
{
}

uint32_t ImuAdis16488::calculateBufferSize(){
	//only use const variables here
	//return	(ImuAdis16488Defaults::NUM_OF_MSGS_IN_PACKAGE*ImuAdis16488Defaults::MSG_SIZE);
	return	(ImuAdis16488Defaults::MSG_SIZE);
}

// set calibration
bool ImuAdis16488::setCalibration(CalibBase::Ptr calibration)
{
	// TODO set calibration
	return true;
}


bool ImuAdis16488::init()
{
	return true;
}


void ImuAdis16488::processMeasurements() {
	while (1) {

		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement meas = Sensor::measurement_queue_.pop();

		//VISENSOR_DEBUG("in imu_adis thread %d\n", meas.buffersize);

		//checkTimestamp(meas.timestamp);

		//		//TODO adapt
		// create new shared pointer for the imu message
		ViImuMsg::Ptr imu_msg_ptr(new ViImuMsg);

		// Add imu information to the msg
		imu_msg_ptr->imu_id = Imu::imu_id_;
		imu_msg_ptr->timestamp=meas.timestamp;

		// get imu data
		getGyro(meas.data,&imu_msg_ptr->gyro[0]);
		getAcc(meas.data,&imu_msg_ptr->acc[0]);
		getMag(meas.data,&imu_msg_ptr->mag[0]);
		getBaro(meas.data,&imu_msg_ptr->baro);

//		// run the imu callback
//		// only run callback if callback is initialized
//		if (Imu::_user_callback)
//			Imu::_user_callback(imu_msg_ptr);

		Sensor::publishImu(imu_msg_ptr);

		delete[] meas.data;
	}
}

//void ImuAdis16488::allocateBuffer()
//{
//	int buffer_size=(ImuAdis16488Defaults::NUM_OF_MSGS_IN_PACKAGE*_msg_size+8);
//	Sensor::_buffer = new unsigned char[buffer_size];
//	Sensor::_buffer_size=buffer_size;
//	Sensor::_bytes_in_buffer=0;
//	Imu::_num_of_completed_msgs=0;
//}
//
//int ImuAdis16488::processData(uint8_t const * data,int nBytesReceived)
//{
//	// get data from FPGA
//	memcpy(&Sensor::_buffer[Sensor::_bytes_in_buffer], data, nBytesReceived);
//
//	Sensor::_bytes_in_buffer+=nBytesReceived;
//
//
//	if(getNumOfNewMsgs()>0)
//		newImuMsgs();
//
//	if(Sensor::_bytes_in_buffer==Sensor::_buffer_size)
//		resetBuffer();
//
//	return 0;
//}

ViConfigMsg ImuAdis16488::getConfigParam(std::string cmd, uint16_t value)
{
	// TODO: implement this
	ViConfigMsg config;
	config.sensorId=0;
	config.devAdress=0;
	config.reg=0;
	config.val=0;
	config.comType=0;
	config.valChanged=false;
	return config;
}

void ImuAdis16488::newImuMsgs()
{
//	int i;
//
//	int temp=getNumOfNewMsgs();
//
//	for(i=0;i<temp;i++)
//	{
//		// create new shared pointer for the imu message
//		ViImuMsg::Ptr imu_msg_ptr(new ViImuMsg);
//
//		// Add imu information to the msg
//		imu_msg_ptr->imu_id=Imu::_imu_id;
////		imu_msg_ptr->timestamp=getTimestamp();
//
//		// get imu data
//		getGyro(&imu_msg_ptr->gyro[0]);
//		getAcc(&imu_msg_ptr->acc[0]);
//		getMag(&imu_msg_ptr->mag[0]);
//		getBaro(&imu_msg_ptr->baro);
//
//		// increment received imu message counter
//		incrementMsgCounter();
//
//		// run the imu callback
//		// only run callback if callback is initialized
//		if(Imu::_user_callback)
//			Imu::_user_callback(imu_msg_ptr);
//	}
}

//int ImuAdis16488::getNumOfNewMsgs()
//{
//	int unused_bytes=Sensor::_bytes_in_buffer-(Imu::_num_of_completed_msgs*_msg_size)-8;
//	return floor(unused_bytes/(_msg_size));
//}


//void ImuAdis16488::resetBuffer()
//{
//	Sensor::_bytes_in_buffer=0;
//	Imu::_num_of_completed_msgs=0;
//}



void ImuAdis16488::getGyro(uint8_t * buffer, double * gyro)
{
	int i,current_pos;
	int temp;
	for(i=0;i<3;i++)
	{
		current_pos=4*i;
		temp=(int)((buffer[current_pos+0]<<24) | (buffer[current_pos+1]<<16) | (buffer[current_pos+2]<<8) | (buffer[current_pos+3]<<0));
		gyro[i]=(double)temp;
		gyro[i] = gyro[i] * 450.0/22500.0/65536.0/180.0*M_PI;		// JN 04052012: Changed to SI units.
	}
}

void ImuAdis16488::getAcc(uint8_t * buffer, double * acc)
{
	int i,current_pos;
	int temp;

	for(i=0;i<3;i++)
	{
		current_pos=3*4+4*i;
		temp=(int)((buffer[current_pos+0]<<24) | (buffer[current_pos+1]<<16) | (buffer[current_pos+2]<<8) | (buffer[current_pos+3]<<0));

		if((temp * 18.0/22500.0/65536.0*STANDARD_GRAVITY)<-200)
			VISENSOR_DEBUG("wrong imu acc value id: %i value raw: %i value: %f \n", i, temp,(temp * 18.0/22500.0/65536.0*STANDARD_GRAVITY));

		acc[i]=(double)temp;
		acc[i] = acc[i] * 18.0/22500.0/65536.0*STANDARD_GRAVITY;	// JN 04052012: Changed to SI units.
	}
}

void ImuAdis16488::getMag(uint8_t * buffer, double * mag)
{
	int i,current_pos;
	int16_t temp;

	for(i=0;i<3;i++)
	{
		current_pos=6*4+2*i;
		temp=(int)((buffer[current_pos+0]<<8) | (buffer[current_pos+1]<<0));
		mag[i]=(double)temp;
		mag[i] = mag[i] * 0.0001;					// JN 04052012: Changed notation, still in gauss.
	}
}

void ImuAdis16488::getBaro(uint8_t * buffer, double * baro)
{
	int current_pos;
	int temp;

	current_pos=6*4+3*2;
	temp=(int)((buffer[current_pos+0]<<24) | (buffer[current_pos+1]<<16) | (buffer[current_pos+2]<<8) | (buffer[current_pos+3]<<0));
	*baro = (double)temp;
	*baro = *baro * 1.31068/32767.0/65536.0;				// JN 04052012: = 40uBar per LSM, hence in bar.
}

//void ImuAdis16488::incrementMsgCounter()
//{
//	Imu::_num_of_completed_msgs++;
//}


} //namespace visensor
