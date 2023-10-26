/*
 * MoveToEncoderAndStopProcess.hpp
 *
 *  Created on: 26 окт. 2023 г.
 *      Author: Pavel
 */

#pragma once

#include <Process.h>
#include <Motor.h>
#include <PID.h>

#include <memory>

namespace ev3 {

class MoveToEncoderAndStopProcess: public Process {
public:
	/*
	 * Запускает мотор в движение до определённого значения энкодера. Останавливает мотор по завершении.
	 */
	MoveToEncoderAndStopProcess(MotorPtr motor, int targetEncoder, int power, std::shared_ptr<PID> pid = nullptr);
	virtual ~MoveToEncoderAndStopProcess() = default;

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	void setPID(std::shared_ptr<PID> pid);

protected:
	MotorPtr motor;
	int targetEncoder;
	int power;
	std::shared_ptr<PID> pid;

};

} /* namespace ev3 */
