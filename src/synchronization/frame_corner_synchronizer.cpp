/*
 * FrameCornerSynchronizer.cpp
 *
 *  Created on: Nov 30, 2013
 *      Author: pascal
 */
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#include "synchronization/frame_corner_synchronizer.hpp"

FrameCornerSynchronizer::FrameCornerSynchronizer() {
}
FrameCornerSynchronizer::~FrameCornerSynchronizer() {
}

void FrameCornerSynchronizer::addFrame(ViFrame::Ptr frame) {
  boost::mutex::scoped_lock lock(mutex_);
  while (corner_queue_.empty())
    cond_.wait(lock);

  if (frame->timestamp < corner_queue_.front()->timestamp) {
    VISENSOR_DEBUG(
        "no matching corners found, publishing image without corners!!!\n");
    frame->useCorners = false;
    return;
  }

  while (frame->timestamp > corner_queue_.front()->timestamp) {
    corner_queue_.pop();
    if (corner_queue_.empty())
      cond_.wait(lock);
  }

  if (frame->timestamp != corner_queue_.front()->timestamp) {
    VISENSOR_DEBUG("pair NOT found: this should not happen!!!\n");
    return;
  }
  ViCorner::Ptr corner = corner_queue_.front();
  corner_queue_.pop();

  if (user_callback_)
    user_callback_(frame, corner);
}

void FrameCornerSynchronizer::addCorner(ViCorner::Ptr corner) {
  {
    boost::mutex::scoped_lock lock(mutex_);
    corner_queue_.push(corner);
  }
  cond_.notify_one();
}

void FrameCornerSynchronizer::setUserCallback(
    boost::function<
        void(ViFrame::Ptr, ViCorner::Ptr)> callback) {
  user_callback_ = callback;
}
