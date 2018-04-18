#include "SerialPort.hpp"

#include <iostream>

std::ostream& operator<<(std::ostream&, const std::vector<uint8_t>&);

const int SerialPort::BUFFER_SIZE = 2048;

SerialPort::SerialPort(boost::asio::io_service& _ioService,
	const std::string& port, int baud, const ReceiveHandler& _recvHandler)
	:	ioService(_ioService)
	, comPort(ioService)
  , recvHandler{_recvHandler}
	,	recvBuffer(BUFFER_SIZE) {
	
	try {
		configureComPort(port, baud);
		startListening();
	}
	catch(const std::exception& e) {
		std::cout << "[Error] Unable to open serial device: " << e.what() << std::endl;
	}
}

bool SerialPort::connected() const {
  return comPort.is_open();
}

void SerialPort::configureComPort(const std::string& port, int baud) {
	comPort.open(port);

	if(!comPort.is_open()) {
		throw std::runtime_error("SerialPort: Unable to open "
			+ port);
	}

	//Configure with no flow control, no parity, 1 stop bit,
	//8 bit character size
	comPort.set_option(boost::asio::serial_port_base::baud_rate(baud));
	comPort.set_option(boost::asio::serial_port_base::flow_control(
		boost::asio::serial_port_base::flow_control::none));
	comPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	comPort.set_option(boost::asio::serial_port_base::stop_bits(
		boost::asio::serial_port_base::stop_bits::one));
	comPort.set_option(boost::asio::serial_port_base::character_size(8));

	startListening();
}

void SerialPort::cbSend(const boost::system::error_code& error,
	size_t bytesTransferred [[gnu::unused]]) {
	
	if(error) {
		//std::cout << "[Error] SerialPort::cbSend: " << error.message() << std::endl;
	}
	
	txList.pop_front();
}

void SerialPort::cbReceive(const boost::system::error_code& error,
	size_t bytesTransferred) {

	if(error) {
		std::cout << "[Error] SerialPort::cbReceive: " << error.message() << std::endl;
	}
	else if(bytesTransferred > 0) {
		recvHandler({recvBuffer.begin(), recvBuffer.begin() + bytesTransferred});
	}

	startListening();
}

void SerialPort::startListening() {
	boost::asio::async_read(comPort,
		boost::asio::buffer(recvBuffer),
		boost::asio::transfer_at_least(1),
		[this](const boost::system::error_code& error,
			size_t bytesTransferred) {
				cbReceive(error, bytesTransferred);
			});
}

void SerialPort::send(std::vector<uint8_t>&& data) {
	txList.emplace_back(std::move(data));

	boost::asio::async_write(comPort, boost::asio::buffer(txList.back()),
		[this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
			cbSend(ec, bytes_transferred);
		});
}
