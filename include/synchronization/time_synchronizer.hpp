/*
 * time_synchronizer.hpp
 *
 *  Created on: Apr 25, 2012
 *      Author: burrimi
 *
 *
 *
 * Time is in nanoseconds!!!
 */

#ifndef TIME_SYNCHRONIZER_HPP_
#define TIME_SYNCHRONIZER_HPP_

#include <config/config.hpp>

namespace TimeSynchronizerValues {
const double TIME_KALMAN_GAIN = 0.001;
const uint32_t NUM_OF_MEASUREMENTS = 5;
const uint32_t SKIP_FIRST_N = 20;
}

class TimeSynchronizer {
 private:

  int64_t _offset_tracker;
  uint64_t _initial_offset;
  uint64_t _previous_timestamp;
  uint32_t _counter;
  int64_t _offset;
  uint32_t unusable_frames_counter_;

  uint64_t _timeFpgaAtUpdate;  // this time is needed to assign consistent timestamps to messages that
  // arrive after the update, but were captured by the fpga before the update
  uint64_t _offsetBeforeUpdate;

 public:
  TimeSynchronizer();
  void init(uint64_t time_fpga);
  void init(uint64_t time_pc, uint64_t time_fpga);
  bool _synchronized;
  void updateTime(uint64_t time_fpga);
  void updateTime(uint64_t time_pc, uint64_t time_fpga);
  uint64_t getSynchronizedTime(uint64_t time_fpga);
  uint64_t getSystemTime();
  bool isSynchronized() {
    return _synchronized;
  }
  ;
};

#endif /* TIME_SYNCHRONIZER_HPP_ */
