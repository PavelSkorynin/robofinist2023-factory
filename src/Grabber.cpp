#include "Grabber.h"

#include "processes.h"

Grabber::Grabber(std::shared_ptr<ev3::Motor> motor)
: motor(motor), pid(std::make_shared<ev3::PID>(0.3f, 0.001f, 1.2f)) {

}

std::shared_ptr<ev3::Process> Grabber::initialize() {
	return std::make_shared<ev3::StopProcess>(motor)
			>> std::make_shared<ev3::SetPowerProcess>(motor, -50)
			>> std::make_shared<ev3::WaitTimeProcess>(0.5f)
			>> std::make_shared<ev3::LambdaProcess>([&](ev3::time_t timestamp) {
		motor->resetEncoder();
		return false;
	});
}

std::shared_ptr<ev3::Process> Grabber::open() {
	return std::make_shared<ev3::StopProcess>(motor)
			>> std::make_shared<ev3::SetPowerProcess>(motor, 50)
			>> std::make_shared<ev3::WaitTimeProcess>(0.5f)
			>> std::make_shared<ev3::SetPowerProcess>(motor, 20)
			>> std::make_shared<ev3::WaitTimeProcess>(0.2f)
			;
}

std::shared_ptr<ev3::Process> Grabber::halfOpen() {
	auto moveProcess = std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, 20, 50, pid);
	moveProcess->setPowerThreshold(2);
	moveProcess->setEncoderThreshold(2);
	return std::make_shared<ev3::StopProcess>(motor)
			>> (moveProcess & std::make_shared<ev3::WaitTimeProcess>(1.0f))
			>> std::make_shared<ev3::StopProcess>(motor);
}

std::shared_ptr<ev3::Process> Grabber::close() {
	return std::make_shared<ev3::StopProcess>(motor)
			>> std::make_shared<ev3::SetPowerProcess>(motor, -50);
}
