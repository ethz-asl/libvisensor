/*
 * FileHandler.hpp
 *
 *  Created on: Nov 22, 2013
 *      Author: pascal
 */

#ifndef FILEHANDLER_HPP_
#define FILEHANDLER_HPP_

#include <config/config.hpp>
#include <boost/asio.hpp>

class FileTransfer {
 public:
  FileTransfer(boost::asio::ip::tcp::socket& socket);
  virtual ~FileTransfer();

  void sendFile(std::string& local_path, std::string& remote_path);
  void receiveFile(std::string& local_path, std::string& remote_path);

 private:
  std::string sendPathString(std::string string);

  boost::asio::ip::tcp::socket& socket_;
};

#endif /* FILEHANDLER_HPP_ */
