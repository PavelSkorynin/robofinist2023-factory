#pragma once

#include <Motor.h>
#include <Process.h>
#include <PID.h>

#include <memory>

class Crane final {
public:
	Crane(std::shared_ptr<ev3::Motor> motor);
	virtual ~Crane() = default;

	std::shared_ptr<ev3::Process> up();
	std::shared_ptr<ev3::Process> down();
	std::shared_ptr<ev3::Process> freeToMove();

private:
	std::shared_ptr<ev3::Motor> motor;
	std::shared_ptr<ev3::PID> pid;
};
