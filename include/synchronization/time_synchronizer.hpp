/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
