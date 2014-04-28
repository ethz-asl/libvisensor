/*
 * sensor_calibration.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: root
 */
#include "sensor.hpp"


namespace visensor
{

CalibProvider::CalibProvider(uint32_t fpgaId)
{

	createCalibrationMaps();

	if(_calibrationMaps.find(fpgaId)==_calibrationMaps.end())
	{
		VISENSOR_DEBUG("no calibration found for fpgaId %u\n",fpgaId);
		return;
	}
	_calibrationMap=_calibrationMaps.at(fpgaId);
}

void CalibProvider::setCalibration(Sensor::Ptr sensor)
{
	uint32_t sensorId=sensor->getId();

	if(_calibrationMap.find(sensorId)==_calibrationMap.end())
	{
		VISENSOR_DEBUG("no calibration found for sensorId %u\n",sensorId);
		return;
	}

	sensor->setCalibration(_calibrationMap.at(sensorId));
}

void CalibProvider::createCalibrationMaps()
{
	boost::shared_ptr<CalibImuMpu6000> tempCalib (new CalibImuMpu6000);

	tempCalib->setMisalignementG(0.00109,0.00360,0.01633);
	tempCalib->setMisalignementA(-0.00115,0.00137,0.00092);
	tempCalib->setScaleG(1.00377,0.99688,1.00473);
	tempCalib->setScaleA(1.00218,1.00008,1.01080);

	addCalibrationToMaps(2,5,tempCalib);

	boost::shared_ptr<CalibImuMpu6000> tempCalib1 (new CalibImuMpu6000);

	tempCalib1->setMisalignementG(0,0,0);
	tempCalib1->setMisalignementA(0,0,0);
	tempCalib1->setScaleG(1,1,1);
	tempCalib1->setScaleA(1,1,1);

	addCalibrationToMaps(3,5,tempCalib1);

}

// this function creates a calibration map for each fpga id and adds the calibration to the submap of the fpga
void CalibProvider::addCalibrationToMaps(uint32_t fpgaId, uint32_t sensorId, CalibBase::Ptr calib)
{
	if(_calibrationMaps.find(fpgaId)==_calibrationMaps.end())
	{
		// create new calibration map for the fpgaId
		std::map<uint32_t,CalibBase::Ptr> tempCalibrationMap;
		tempCalibrationMap.insert( std::pair<uint32_t,CalibBase::Ptr> (sensorId, calib));
		_calibrationMaps.insert( std::pair<uint32_t,std::map<uint32_t,CalibBase::Ptr> >(fpgaId, tempCalibrationMap));
	}else{
		// if fpgaId exists add calib to the corresponding calibration map
		_calibrationMaps.at(fpgaId).insert( std::pair<uint32_t,CalibBase::Ptr> (sensorId, calib));
	}
}

} //namespace visensor
