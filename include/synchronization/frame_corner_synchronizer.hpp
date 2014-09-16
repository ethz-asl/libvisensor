/*
 * FrameCornerSynchronizer.hpp
 *
 *  Created on: Nov 30, 2013
 *      Author: pascal
 */

#ifndef FRAMECORNERSYNCHRONIZER_HPP_
#define FRAMECORNERSYNCHRONIZER_HPP_

#include <config/config.hpp>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#include "visensor/visensor_datatypes.hpp"

using namespace visensor;

class FrameCornerSynchronizer {
 public:
  FrameCornerSynchronizer();
  ~FrameCornerSynchronizer();

  void addFrame(ViFrame::Ptr frame);
  void addCorner(ViCorner::Ptr corner);
  void setUserCallback(
      boost::function<void (ViFrame::Ptr, ViCorner::Ptr)> callback);

 private:
  boost::function< void (ViFrame::Ptr, ViCorner::Ptr)> user_callback_;
  mutable boost::mutex mutex_;
  boost::condition_variable cond_;
  std::queue<ViCorner::Ptr> corner_queue_;
};

#endif /* FRAMECORNERSYNCHRONIZER_HPP_ */
