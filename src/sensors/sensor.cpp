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
 *     http://www.apache.org/licenses/LICENSE-2.0
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

#include <boost/shared_ptr.hpp>

#include <sensors/sensor.hpp>

namespace visensor
{

Sensor::Sensor(SensorSettings sensor_settings, boost::shared_ptr<ConfigConnection> config_connection):
    settings_(sensor_settings),
    config_connection_(config_connection),
    prev_timestamp_(0),
    allowed_timediff_(0)
{
}

Sensor::~Sensor()
{
}

void Sensor::writeRequest(const ViConfigMsg configMsg)
{
  config_connection_->writeConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg,configMsg.val, configMsg.comType);
  return;
}

void Sensor::readRequest(ViConfigMsg& configMsg)
{
  config_connection_->readConfig(configMsg.sensorId,configMsg.devAdress,configMsg.reg, configMsg.val, configMsg.comType);
  return;
}

void Sensor::checkTimestamp(uint64_t timestamp)
{
	if(prev_timestamp_==0)
	{
		prev_timestamp_=timestamp;
		return;
	}

	if(timestamp-prev_timestamp_>allowed_timediff_)
		VISENSOR_DEBUG("sensor %d frame missed, timediff: %ld ns\n",stream_id(),timestamp-prev_timestamp_);
	prev_timestamp_=timestamp;
}

int Sensor::getMeasurementBufferSize() const
{
  return settings_.measurementBufferSize;
}

void Sensor::addMeasurement(const Measurement::Ptr meas){
  measurement_queue_.push(meas);
}

uint32_t Sensor::getTimeBetweenMsgs() const
{
	// avoid division by 0, just return 0 if rate is not set yet
	if(settings_.rate==0)
		return 0;

	return 1000000000/settings_.rate;
}

int Sensor::getNumOfMsgsInPackage() const
{
	return settings_.numOfMsgsInPackage;
}

// start sensor with given rate
bool Sensor::startSensor(uint32_t rate)
{
	settings_.rate=rate;
	settings_.active=true;
	allowed_timediff_=1.5*getTimeBetweenMsgs();
	return true;
}

// stop sensor
bool Sensor::stopSensor()
{
	settings_.active=false;
	return true;
}

}//namespace visensor
