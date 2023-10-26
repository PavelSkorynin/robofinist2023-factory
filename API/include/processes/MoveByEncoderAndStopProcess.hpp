/*
 * MoveByEncoderAndStopProcess.hpp
 *
 *  Created on: 30 янв. 2020 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include "MoveToEncoderAndStopProcess.hpp"

namespace ev3 {

class MoveByEncoderAndStopProcess: public MoveToEncoderAndStopProcess {
public:
	/*
	 * Запускает мотор в движение на определённое число градусов. Останавливает мотор по завершении.
	 */
	MoveByEncoderAndStopProcess(MotorPtr motor, int deltaEncoder, int power, std::shared_ptr<PID> pid = nullptr);

	virtual void onStarted(time_t secondsFromStart) override;
protected:
	int deltaEncoder;
};

} /* namespace ev3 */

