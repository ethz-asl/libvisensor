#include <boost/function.hpp>

#include <sensors/imu.hpp>

namespace visensor
{

void Imu::setUserCallback(boost::function<void (ViImuMsg::Ptr, ViErrorCode)> callback)
{
	user_callback_=callback;
}

void Imu::publishImuData(ViImuMsg::Ptr imu, ViErrorCode error)
{
  /////////////////////////////
  //
  // WORKAROUND FOR FPGA BUG
  //
  // some bug in the imu pipeline causes ~4 old imu measurements to be stuck
  // in some buffer. these measurements are released at reconnect.
  //
  // therefore we drop the first 4 messages, until we have an fpga fix
  //
  /////////////////////////////

  //discard the first 4 real imu messages (1 imu message = 10 imu measurements)
  static int drop_counter = 41;
  if(drop_counter > 0)
  {
    //only real frames count (= no error)
    if(error==visensor::ViErrorCodes::NO_ERROR)
      drop_counter--;
    return;
  }

  // END WORKAROUND FOR FPGA BUG
  /////////////////////////////

  //publish
  if(user_callback_)
    user_callback_(imu, error);
}

} //namespace visensor
