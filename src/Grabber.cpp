#include "Grabber.h"

#include "processes.h"

Grabber::Grabber(std::shared_ptr<ev3::Motor> motor)
: motor(motor) {

}

std::shared_ptr<ev3::Process> Grabber::initialize() {
	return std::make_shared<ev3::SetPowerProcess>(motor, -50)
			>> std::make_shared<ev3::WaitTimeProcess>(0.5f)
			>> std::make_shared<ev3::LambdaProcess>([&](ev3::time_t timestamp) {
		motor->resetEncoder();
		return false;
	});
}

std::shared_ptr<ev3::Process> Grabber::open() {
	return std::make_shared<ev3::SetPowerProcess>(motor, 50)
			>> std::make_shared<ev3::WaitTimeProcess>(0.5f);
}

std::shared_ptr<ev3::Process> Grabber::halfOpen() {
	return std::make_shared<ev3::MoveToEncoderAndStopProcess>(motor, 12, 50)
			& std::make_shared<ev3::WaitTimeProcess>(0.5f);
}

std::shared_ptr<ev3::Process> Grabber::close() {
	return std::make_shared<ev3::SetPowerProcess>(motor, -50)
			>> std::make_shared<ev3::WaitTimeProcess>(0.5f);
}
