#include "Crane.h"

#include <processes.h>

const int FULL_RANGE = 6200;

Crane::Crane(std::shared_ptr<ev3::Motor> motor)
: motor(std::move(motor))
, pid(std::make_shared<ev3::PID>(1.0f, 0.00001f, 2.0f))
{
}

std::shared_ptr<ev3::Process> Crane::up() {
	pid->reset();
	auto moveProcess = std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -FULL_RANGE, 100, pid);
	moveProcess->setEncoderThreshold(10);
	moveProcess->setPowerThreshold(5);
	return (moveProcess
			& std::make_shared<ev3::WaitTimeProcess>(5.0f)) >> ev3::StopProcess(motor);
}

std::shared_ptr<ev3::Process> Crane::down() {
	pid->reset();
	auto moveProcess = std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, 0, 100, pid);
	moveProcess->setEncoderThreshold(10);
	moveProcess->setPowerThreshold(5);
	return (moveProcess
			& std::make_shared<ev3::WaitTimeProcess>(5.0f)) >> ev3::StopProcess(motor);
}

std::shared_ptr<ev3::Process> Crane::freeToMove() {
	pid->reset();
	auto moveProcess = std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, -1300, 100, pid);
	moveProcess->setEncoderThreshold(10);
	moveProcess->setPowerThreshold(5);
	return (moveProcess
			& std::make_shared<ev3::WaitTimeProcess>(5.0f)) >> ev3::StopProcess(motor);
}
