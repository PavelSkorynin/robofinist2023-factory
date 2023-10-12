/*
 * WaitEncoderProcess.hpp
 *
 *  Created on: 28 янв. 2020 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>
#include <Motor.h>

namespace ev3 {

class WaitEncoderProcess: public virtual Process {
public:
	WaitEncoderProcess(MotorPtr motor, int targetEncoder);
	virtual ~WaitEncoderProcess() = default;

	virtual void onStarted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

protected:
	MotorPtr motor;
	int targetEncoder;
	int startEncoder = 0;
};

} /* namespace ev3 */
