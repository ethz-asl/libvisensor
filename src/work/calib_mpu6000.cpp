/*
 * calib_mpu6000.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: burrimi
 */


#include "sensor_calibration.hpp"


namespace visensor
{
	void CalibImuMpu6000::setScaleA(double a0,double a1, double a2)
	{
		_scaleA.at(0)=a0;
		_scaleA.at(1)=a1;
		_scaleA.at(2)=a2;
	}

	void CalibImuMpu6000::setScaleG(double a0,double a1, double a2)
	{
		_scaleG.at(0)=a0;
		_scaleG.at(1)=a1;
		_scaleG.at(2)=a2;
	}

	void CalibImuMpu6000::setMisalignementA(double a0,double a1, double a2)
	{
		_misalignementA.at(0)=a0;
		_misalignementA.at(1)=a1;
		_misalignementA.at(2)=a2;
	}

	void CalibImuMpu6000::setMisalignementG(double a0,double a1, double a2)
	{
		_misalignementG.at(0)=a0;
		_misalignementG.at(1)=a1;
		_misalignementG.at(2)=a2;
	}

	std::vector<std::vector<double> > CalibImuMpu6000::getScaleMisalignementInverseG()
	{
		std::vector<double> temp(3,0.0);
		std::vector<std::vector<double> > scaleMisalignementInverseG(3,temp);

		//scaleMisalignementInverse = inv(Scale*Misalignement)
		scaleMisalignementInverseG.at(0).at(0)=1/_scaleG[0];
		scaleMisalignementInverseG.at(1).at(0)=-_misalignementG[0]/_scaleG[0];
		scaleMisalignementInverseG.at(1).at(1)=1/_scaleG[1];
		scaleMisalignementInverseG.at(2).at(0)=(_misalignementG[0]*_misalignementG[2]-_misalignementG[1])/_scaleG[0];
		scaleMisalignementInverseG.at(2).at(1)=-_misalignementG[2]/_scaleG[1];
		scaleMisalignementInverseG.at(2).at(2)=1/_scaleG[2];

		return scaleMisalignementInverseG;
	}

	std::vector<std::vector<double> > CalibImuMpu6000::getScaleMisalignementInverseA()
	{

		std::vector<double> temp(3,0.0);
		std::vector<std::vector<double> > scaleMisalignementInverseA(3,temp);

		//scaleMisalignementInverse = inv(Scale*Misalignement)
		scaleMisalignementInverseA.at(0).at(0)=1/_scaleA[0];
		scaleMisalignementInverseA.at(1).at(0)=-_misalignementA[0]/_scaleA[0];
		scaleMisalignementInverseA.at(1).at(1)=1/_scaleA[1];
		scaleMisalignementInverseA.at(2).at(0)=(_misalignementA[0]*_misalignementA[2]-_misalignementA[1])/_scaleA[0];
		scaleMisalignementInverseA.at(2).at(1)=-_misalignementA[2]/_scaleA[1];
		scaleMisalignementInverseA.at(2).at(2)=1/_scaleA[2];

//		std::vector<std::vector<double> > scaleMisalignementInverseG={{1/_scaleA[0],0,0},
//				{-_misalignementA[0]/_scaleA[0],1/_scaleA[1],0},
//				{(_misalignementA[0]*_misalignementA[2]-_misalignementA[1])/_scaleA[0],-_misalignementA[2]/_scaleA[1],1/_scaleA[2]}};
		return scaleMisalignementInverseA;
	}



	// todo implement this
	void CalibImuMpu6000::getCalibratedMeasurement(ViImuMsg::Ptr imu_ptr)
	{

	}


} //namespace visensor
