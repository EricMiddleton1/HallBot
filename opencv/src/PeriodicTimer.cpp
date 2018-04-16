#include "PeriodicTimer.hpp"

#include <stdexcept>
#include <iostream>

PeriodicTimer::PeriodicTimer(boost::asio::io_service& _ioService,
	const std::chrono::microseconds& _period, const TimerHandler& _handler)
	:	running{true}
	,	ioService{_ioService}
	,	period{_period}
	,	handler{_handler}
	,	timer{ioService} {
	
	if(!handler) {
		throw std::runtime_error("PeriodicTimer: Invalid handler");
	}

	timer.expires_from_now(period);
	timer.async_wait([this](const boost::system::error_code& error) {
		cbTimer(error);
	});
}

PeriodicTimer::~PeriodicTimer() {
	running = false;
	timer.cancel();
}

void PeriodicTimer::cbTimer(const boost::system::error_code& error) {
	if(!error) {
		handler();
	}
	else if(error == boost::asio::error::operation_aborted) {
		return;
	}
	else {
		std::cout << "[Error] PeriodicTimer: cbTimer: " << error.message() << std::endl;
	}

	resetTimer();
}

void PeriodicTimer::resetTimer() {
	if(running) {
		timer.expires_from_now(timer.expires_from_now() + period);
		timer.async_wait([this](const boost::system::error_code& error) {
			cbTimer(error);
		});
	}
}
