#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <list>
#include <functional>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class SerialPort
{
public:
  using ReceiveHandler = std::function<void(std::vector<uint8_t>)>;

  SerialPort(boost::asio::io_service& ioService, const std::string& port, int baud,
    const ReceiveHandler& recvHandler);

  void send(std::vector<uint8_t>&& data);

private:
  static const int BUFFER_SIZE;

  void cbSend(const boost::system::error_code& error,
              size_t bytesTransferred);
  void cbReceive(const boost::system::error_code& error,
              size_t bytesTransferred);

  void configureComPort(const std::string& port, int baud);

  void startListening();
  
  boost::asio::io_service& ioService;
  boost::asio::serial_port comPort;

  ReceiveHandler recvHandler;
  
  std::vector<uint8_t> recvBuffer;

  std::list<std::vector<uint8_t>> txList;
};
