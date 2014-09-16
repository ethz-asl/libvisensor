#include <boost/function.hpp>

#include "visensor/visensor_datatypes.hpp"
#include "sensors/corner_detector.hpp"

namespace visensor {

void CornerDetector::setCornerCallback(boost::function<void(ViCorner::Ptr, ViErrorCode)> callback) {
  user_callback_ = callback;
}

void CornerDetector::publishCornerData(ViCorner::Ptr& frame, ViErrorCode error) {
  if (user_callback_)
    user_callback_(frame, error);
}

}  // namespace visensor
