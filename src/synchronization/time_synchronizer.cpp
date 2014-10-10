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

#include <sys/time.h>
#include "synchronization/time_synchronizer.hpp"

using namespace TimeSynchronizerValues;

TimeSynchronizer::TimeSynchronizer():_synchronized(false)
{
	_offset_tracker=0;
	_initial_offset=0;
	_previous_timestamp=0;
	_counter=0;
	_offset=0;
	unusable_frames_counter_=0;
	_timeFpgaAtUpdate=0;
	_offsetBeforeUpdate=0;
}

void TimeSynchronizer::init(uint64_t time_fpga)
{
	init(getSystemTime(), time_fpga);
}


void TimeSynchronizer::init(uint64_t time_pc, uint64_t time_fpga)
{
	_initial_offset=time_pc-time_fpga;
	_synchronized=true;
}

void TimeSynchronizer::updateTime(uint64_t time_fpga)
{
	updateTime(getSystemTime(), time_fpga);
}

void TimeSynchronizer::updateTime(uint64_t time_pc, uint64_t time_fpga)
{
//WARNING(schneith): leica code assumes that the clock offset/skew is fixed for all msgs

// detect fpga time wrap around
//if(time_fpga<_previous_timestamp)
//{
//  _initial_offset += (uint64_t)0xFFFFFFFF+1;
//}
//_previous_timestamp = time_fpga;

//	// ignore first 10 frames
//	if(unusable_frames_counter_<SKIP_FIRST_N)
//	{
//		unusable_frames_counter_++;
//		return;
//	}
//
//
//	_offset_tracker+=(time_pc-_initial_offset-time_fpga);
//	++_counter;
//
//	if(_counter<NUM_OF_MEASUREMENTS)
//		return;
//
//
//	//VISENSOR_DEBUG("offset tracker: %lu offset time: %lu\n",(time_pc-_initial_offset)/1000000, time_fpga/1000000);
//	double diff;
//	diff=(double)_offset_tracker/NUM_OF_MEASUREMENTS-(double)_offset;
//	//diff=(double)_offset_tracker-(double)_offset;
//
//	// update offset
//	if(synchronized_)
//	{
////		_offsetBeforeUpdate=_offset;
////		_timeFpgaAtUpdate=time_fpga;
////		_offset+=(int64_t)(TIME_KALMAN_GAIN*diff);
//	}
//	else // on first run don't use kalman gain
//	{
//		_offset=(int64_t)(diff);
//		_offsetBeforeUpdate=_offset;
//		_synchronized=true;
//	}
//
//	VISENSOR_DEBUG("offset tracker: %li update: %li timediff fpga-pc: %li\n",_offset, (int64_t)(TIME_KALMAN_GAIN*diff),_offset_tracker-_offset);
//
//	_offset_tracker=0;
//	_counter=0;
}

// returns synchronized time in nanoseconds
uint64_t TimeSynchronizer::getSynchronizedTime(uint64_t time_fpga)
{
  //TODO(schneith): fpga counter will overflow after ~11h (32bit @ 100kHz) --> handle this here?
    return time_fpga+_initial_offset;
/*
	// take old offset for messages captured before the update
	if(time_fpga>_timeFpgaAtUpdate)
		return time_fpga+_initial_offset+_offset;
	else
		return time_fpga+_initial_offset+_offsetBeforeUpdate;
*/
}

// get system time in nanoseconds
uint64_t TimeSynchronizer::getSystemTime()
{
	//timespec ts;
	// clock_gettime(CLOCK_MONOTONIC, &ts); // Works on FreeBSD
	//clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux
	//return (uint64_t)(ts.tv_sec) * 1000000000 + (uint64_t)(ts.tv_nsec);

	 timeval tv;
	 gettimeofday(&tv,NULL);
	 return (uint64_t)tv.tv_sec * 1000000000 + (uint64_t)tv.tv_usec * 1000;
}
