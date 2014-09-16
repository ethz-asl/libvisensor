/*
 * FileHandler.cpp
 *
 *  Created on: Nov 22, 2013
 *      Author: pascal
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
