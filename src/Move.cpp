#include "Move.h"

#include <processes.h>

Move::Move(std::shared_ptr<EV3> eva, std::shared_ptr<Motor> leftMotor, std::shared_ptr<Motor> rightMotor, std::shared_ptr<Sensor> leftLineSensor, std::shared_ptr<Sensor> rightLineSensor)
: eva(std::move(eva))
, leftMotor(std::move(leftMotor))
, rightMotor(std::move(rightMotor))
, leftLineSensor(std::move(leftLineSensor))
, rightLineSensor(std::move(rightLineSensor))
, movePID(std::make_shared<PID>(0.3f, 0.0f, 0.9f))
, power(70)
{

}

void Move::setPower(int power) {
	this->power = power;
}

int Move::getPower() const {
	return power;
}

std::shared_ptr<Process> Move::moveOnLine(int distance, bool stop) {
	if (stop) {
		return std::make_shared<StopOnLineProcess>(leftMotor, rightMotor, leftLineSensor, rightLineSensor, distance, power, movePID);
	} else {
		return std::make_shared<MoveOnLineProcess>(leftMotor, rightMotor, leftLineSensor, rightLineSensor, distance, power, movePID);
	}
}

std::shared_ptr<Process> Move::moveOnLineToCross(int distanceAfterCross, bool stop) {
	auto waitCrossProcess = std::make_shared<WaitCrossProcess>(leftMotor, rightMotor, leftLineSensor, rightLineSensor);
	waitCrossProcess->setMeanThreshold(10);
	auto moveOnToCross = (MoveOnLineProcess(leftMotor, rightMotor, leftLineSensor, rightLineSensor, INT_MAX / 4, power, movePID)
		& waitCrossProcess) >> LambdaProcess([this](float timestamp) {
				eva->playSound(50, 0.1, 0.2);
				return false;
			});
	if (stop) {
		return moveOnToCross >> StopOnLineProcess(leftMotor, rightMotor, leftLineSensor, rightLineSensor, distanceAfterCross, power, movePID);
	} else {
		return moveOnToCross >> MoveOnLineProcess(leftMotor, rightMotor, leftLineSensor, rightLineSensor, distanceAfterCross, power, movePID);
	}
}

std::shared_ptr<Process> Move::moveToCross(int distanceAfterCross, bool stop) {
	auto moveOnToCross = MoveByEncoderOnArcProcess(leftMotor, rightMotor, INT_MAX / 4, INT_MAX / 4, power)
		& WaitCrossProcess(leftMotor, rightMotor, leftLineSensor, rightLineSensor);
	if (stop) {
		return moveOnToCross >> StopByEncoderOnArcProcess(leftMotor, rightMotor, distanceAfterCross, distanceAfterCross, power);
	} else {
		return moveOnToCross >> MoveByEncoderOnArcProcess(leftMotor, rightMotor, distanceAfterCross, distanceAfterCross, power);
	}
}

std::shared_ptr<Process> Move::moveByEncoder(int leftDistance, int rightDistance, bool stop) {
	if (stop) {
		return std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, leftDistance, rightDistance, power);
	} else {
		return std::make_shared<MoveByEncoderOnArcProcess>(leftMotor, rightMotor, leftDistance, rightDistance, power);
	}
}

std::shared_ptr<Process> Move::rotateToLineLeft(int minDistance, bool stop) {
	movePID->reset();
	return MoveByEncoderOnArcProcess(leftMotor, rightMotor, -minDistance, minDistance, power)
					>> (MoveByEncoderOnArcProcess(leftMotor, rightMotor, -INT_MAX/4, INT_MAX/4, std::max(power / 4, 20))
							& WaitLineProcess(leftLineSensor))
					>> alignToLine(stop);

}

std::shared_ptr<Process> Move::rotateToLineRight(int minDistance, bool stop) {
	movePID->reset();
	return MoveByEncoderOnArcProcess(leftMotor, rightMotor, minDistance, -minDistance, power)
					>> (MoveByEncoderOnArcProcess(leftMotor, rightMotor, INT_MAX/4, -INT_MAX/4, std::max(power / 4, 20))
							& WaitLineProcess(rightLineSensor))
					>> alignToLine(stop);
}

std::shared_ptr<Process> Move::alignToLine(bool stop) {
	auto alignProcess = LambdaProcess([this](float timestamp) {
		float delta = rightLineSensor->getValue() - leftLineSensor->getValue();
		rightMotor->setPower(delta / 4);
		leftMotor->setPower(-delta / 4);
		return abs(delta) > 5;
	}) & WaitTimeProcess(0.5f);
	if (stop) {
		return alignProcess >> (StopProcess(leftMotor) | StopProcess(rightMotor));
	}
	return alignProcess;
}
