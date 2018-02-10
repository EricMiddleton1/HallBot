#include <iostream>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include "iRobot.hpp"

const std::string PORT = "/dev/ttyUSB0";

int main() {
  boost::asio::io_service ioService;
  boost::asio::io_service::work ioWork(ioService);

  std::cout << "[Info] Constructing iRobot" << std::endl;
  iRobot bot{ioService, PORT};

  std::cout << "[Info] Starting iRobot" << std::endl;
  bot.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  std::cout << "[Info] Starting wheels" << std::endl;
  bot.setWheels(0, 250);

  std::cout << "[Info] Starting main loop" << std::endl;
  ioService.run();

  std::cout << "[Info] Exiting main loop" << std::endl;
  return 0;
}
