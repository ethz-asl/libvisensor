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

#include <fstream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "networking/file_transfer.hpp"

namespace
{
const std::size_t CHUNK_SIZE = 8096;
}

FileTransfer::FileTransfer(boost::asio::ip::tcp::socket& socket)
: socket_(socket){
}

FileTransfer::~FileTransfer() {
}

void FileTransfer::sendFile(std::string& local_path, std::string& remote_path) {

  sendPathString(remote_path);

  std::ifstream stream(local_path.c_str(), std::ios::binary);
  if (!stream)
  {
    VISENSOR_DEBUG("could not open file: %s\n", local_path.c_str());
    return;
  }

  stream.seekg(0, std::ios::end);
  uint64_t length = stream.tellg();
  stream.seekg(0);

  // file length
  boost::asio::write(socket_, boost::asio::buffer(&length, sizeof(length)));

  // file content
  boost::array<char, CHUNK_SIZE> chunk;

  uint64_t transferred = 0;

  while (transferred != length)
  {
    uint64_t remaining = length - transferred;
    std::size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : static_cast<std::size_t>(remaining);
    stream.read(&chunk[0], chunk_size);
    boost::asio::write(socket_, boost::asio::buffer(chunk, chunk_size));
    transferred += chunk_size;
  }
}

void FileTransfer::receiveFile(std::string& local_path, std::string& remote_path) {

  sendPathString(remote_path);

  std::ofstream stream(local_path.c_str(), std::ios::binary);
  if (!stream)
  {
    VISENSOR_DEBUG( "could not save file: %s\n", local_path.c_str());
    return;
  }

  // file length
  uint64_t length = 0;
  boost::asio::read(socket_, boost::asio::buffer(&length, sizeof(length)));

  // file content
  boost::array<char, CHUNK_SIZE> chunk;

  uint64_t transferred = 0;

  while (transferred != length)
  {
    uint64_t remaining = length - transferred;
    std::size_t chunk_size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : static_cast<std::size_t>(remaining);
    boost::asio::read(socket_, boost::asio::buffer(chunk, chunk_size));
    stream.write(&chunk[0], chunk_size);
    transferred += chunk_size;
  }
}

std::string FileTransfer::sendPathString(std::string string) {
//  // send string length
//  boost::uint64_t string_size = string.size();
//  boost::asio::write(socket_, boost::asio::buffer(&string_size, sizeof(string_size)));
//
//  // send string
//  boost::asio::write(socket_, boost::asio::buffer(&string, sizeof(string_size)));

  boost::asio::streambuf buff;
  std::ostream os(&buff);
  os << string;

  socket_.send(buff.data());  // for example

  return string;
}
