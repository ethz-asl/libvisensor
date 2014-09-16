#include <boost/function.hpp>
#include <boost/foreach.hpp>

#include <sensors/camera.hpp>

namespace visensor {

void Camera::setFrameCallback(boost::function<void(ViFrame::Ptr, ViErrorCode)> callback) {
  user_callbacks_.push_back(callback);
}

void Camera::publishFrameData(ViFrame::Ptr& frame, ViErrorCode error) {
  if (user_callbacks_.empty() == 0) {
    BOOST_FOREACH(
        boost::function<void(ViFrame::Ptr, ViErrorCode)> callback, user_callbacks_) {
      callback(frame, error);
    }
  }
}

}  // namespace visensor
