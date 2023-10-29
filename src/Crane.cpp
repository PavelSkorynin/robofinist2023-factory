#include "Crane.h"

#include <processes.h>

const int FULL_RANGE = 5800;

Crane::Crane(std::shared_ptr<ev3::Motor> motor)
: motor(std::move(motor))
, pid(std::make_shared<ev3::PID>(0.1f, 0, 0.4f))
{
}

std::shared_ptr<ev3::Process> Crane::up() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -FULL_RANGE, 100, pid);
}

std::shared_ptr<ev3::Process> Crane::down() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, 0, 100, pid);
}

std::shared_ptr<ev3::Process> Crane::freeToMove() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -400, 100, pid);
}
