/*
 * MoveByEncoderAndStopProcess.hpp
 *
 *  Created on: 30 янв. 2020 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>
#include <Motor.h>

namespace ev3 {

class MoveByEncoderAndStopProcess: public Process {
public:
	/*
	 * Запускает мотор в движение на определённое число градусов. Останавливает мотор по заверщении.
	 */
	MoveByEncoderAndStopProcess(MotorPtr motor, int deltaEncoder, int power);
	virtual ~MoveByEncoderAndStopProcess() = default;

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;
	virtual void onCompleted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

protected:
	MotorPtr motor;
	int deltaEncoder;
	int power;
	time_t startTime;
};

} /* namespace ev3 */

