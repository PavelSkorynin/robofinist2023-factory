#pragma once

#include <memory>

#include <EV3.h>
#include <Motor.h>
#include <Sensor.h>
#include <PID.h>

using namespace ev3;

class Move final {
public:
	Move(std::shared_ptr<EV3> eva, std::shared_ptr<Motor> leftMotor, std::shared_ptr<Motor> rightMotor, std::shared_ptr<Sensor> leftLineSensor, std::shared_ptr<Sensor> rightLineSensor);

	virtual ~Move() = default;

	void setPower(int power);
	int getPower() const;

	std::shared_ptr<Process> moveOnLine(int distance, bool stop);
	std::shared_ptr<Process> moveOnLineToCross(int distanceAfterCross, bool stop);
	std::shared_ptr<Process> moveToCross(int distanceAfterCross, bool stop);
	std::shared_ptr<Process> moveByEncoder(int leftDistance, int rightDistance, bool stop);
	std::shared_ptr<Process> rotateToLineLeft(int minDistance, bool stop);
	std::shared_ptr<Process> rotateToLineRight(int minDistance, bool stop);
	std::shared_ptr<Process> alignToLine(bool stop);
	std::shared_ptr<Process> rotateLeft(bool stop);
	std::shared_ptr<Process> rotateRight(bool stop);

private:
	std::shared_ptr<EV3> eva;
	std::shared_ptr<Motor> leftMotor;
	std::shared_ptr<Motor> rightMotor;
	std::shared_ptr<Sensor> leftLineSensor;
	std::shared_ptr<Sensor> rightLineSensor;

	std::shared_ptr<PID> movePID;

	int power;
};

