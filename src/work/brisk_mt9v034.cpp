/*
 * brisk_mt9v034.cpp
 *
 *  Created on: Apr 18, 2012
 *      Author: burrimi
 */

#include "brisk_mt9v034.hpp"
#include "visensor_impl.hpp" // needed by forward declaration

namespace visensor
{
BriskMt9v034::BriskMt9v034(int sensor_id,int camera_id, ViSensorDriver::Impl* ViSensorDriver):
				Camera(camera_id,ViCameraConfig(
						BriskMt9v034Defaults::WIDTH, BriskMt9v034Defaults::HEIGHT, BriskMt9v034Defaults::RATE),
						ViSensorDriver,
						SensorSettings(sensor_id,SensorType::SensorType::CORNER_MT9V034,calculateBufferSize(),
								BriskMt9v034Defaults::NUM_OF_MSGS_IN_PACKAGE,
								BriskMt9v034Defaults::USE_CONST_PACKAGE_SIZE,
								BriskMt9v034Defaults::CALIBRATION_SIZE)),
				_config(sensor_id)
{



}

bool BriskMt9v034::init()
{
	return true;
}

// set calibration
bool BriskMt9v034::setCalibration(CalibBase::Ptr calibration)
{
	// TODO set calibration
	return true;
}


uint32_t BriskMt9v034::calculateBufferSize(){
	//only use const variables here
	return	(BriskMt9v034Defaults::MAX_FEATURES*(BriskMt9v034Defaults::SCORE_BYTES+BriskMt9v034Defaults::BRISK_BYTES+4));
}

BriskMt9v034::~BriskMt9v034()
{

}

//the threaded function that works on the queue of finished measurements
void BriskMt9v034::processMeasurements(){
	while(1){


		boost::this_thread::interruption_point();
		//get the newest measurement
		Measurement meas = Sensor::measurement_queue_.pop();

//		VISENSOR_DEBUG("in corner thread %d\n", Camera::_cameraId);

		Sensor::checkTimestamp(meas.timestamp);


//		// create new shared pointer for the corners
		ViCorner::Ptr corner_ptr(new ViCorner());

		corner_ptr->camera_id=Camera::camera_id_;
		corner_ptr->timestamp=meas.timestamp;

		// process corners
		uint32_t pos=0;
		while(pos<meas.bytes_in_buffer)
		{
			ViCornerElem corner;
			if(getBriskElem(&meas.data[pos],corner))
				corner_ptr->corners.push_back(corner);
//			else
//				VISENSOR_DEBUG("pos in buffer: %u \n",meas._bytesInBuffer/8-pos/8);
			pos+=BriskMt9v034Defaults::SCORE_BYTES+BriskMt9v034Defaults::BRISK_BYTES+4;
		}

//		VISENSOR_DEBUG("corners size %lu\n",corner_ptr->corners.size());

		delete[] meas.data;

		Sensor::publishCorner(corner_ptr);
	}
}

bool BriskMt9v034::getBriskElem(uint8_t* buf, ViCornerElem& corner)
{

	corner.x=((buf[0]<<8) | (buf[1]<<0));
	corner.y=((buf[2]<<8) | (buf[3]<<0));
	corner.score=(buf[4]<<24) | (buf[5]<<16) | (buf[6]<<8) | (buf[7]<<0);

	memcpy(corner.descriptor,&buf[8],BriskMt9v034Defaults::BRISK_BYTES);

	if(corner.x>=Camera::config_.width || corner.y>=Camera::config_.height)
	{
		VISENSOR_DEBUG("Sensor::Camera::BriskMt9v034 - invalid corner position x:%u y:%u\n",corner.x,corner.y);
		return false;
	}

//	if(corner.x==0 && corner.y==0)
//	{
//		VISENSOR_DEBUG("Sensor::Camera::CornerMt9v034 - zero corner received score:%u\n",corner.score);
//		return false;
//	}

	return true;
}

ViConfigMsg BriskMt9v034::getConfigParam(std::string cmd, uint16_t value)
{
	return _config.getConfigParam(cmd,value);
}


//void BriskMt9v034::checkTimestamp(uint64_t timestamp)
//{
////	//VISENSOR_DEBUG("camera %d frame missed, timediff: %ld ns\n",_camera_id,Sensor::_timestamp);
////
//	if(_prevTimestamp==0)
//	{
//		_prevTimestamp=timestamp;
//		return;
//	}
//
//	if(timestamp-_prevTimestamp>_allowedTimediff)
//		VISENSOR_DEBUG("corner %d frame missed, timediff: %ld ns\n",Camera::_cameraId,timestamp-_prevTimestamp);
//	_prevTimestamp=timestamp;
//}


} //namespace visensor
