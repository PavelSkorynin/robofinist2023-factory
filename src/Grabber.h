#pragma once

#include "Motor.h"
#include "Process.h"

#include <memory>

class Grabber final {
public:
	Grabber(std::shared_ptr<ev3::Motor> motor);
	virtual ~Grabber() = default;

	std::shared_ptr<ev3::Process> initialize();
	std::shared_ptr<ev3::Process> open();
	std::shared_ptr<ev3::Process> halfOpen();
	std::shared_ptr<ev3::Process> close();

private:
	std::shared_ptr<ev3::Motor> motor;
};
