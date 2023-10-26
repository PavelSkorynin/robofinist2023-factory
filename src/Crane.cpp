#include "Crane.h"

#include <processes.h>

const int FULL_RANGE = 6172;

Crane::Crane(std::shared_ptr<ev3::Motor> motor)
: motor(std::move(motor))
, pid(std::make_shared<ev3::PID>(0.3f, 0, 1.0f))
{
}

std::shared_ptr<ev3::Process> Crane::up() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -FULL_RANGE, 100);
}

std::shared_ptr<ev3::Process> Crane::down() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, 0, 100);
}

std::shared_ptr<ev3::Process> Crane::freeToMove() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -200, 100);
}
