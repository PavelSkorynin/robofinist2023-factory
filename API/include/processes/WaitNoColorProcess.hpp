/*
 * WaitNoColorProcess.hpp
 *
 *  Created on: 6 янв. 2020 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>

#include <Process.h>
#include <Sensor.h>


namespace ev3 {

class WaitNoColorProcess: public virtual Process {
public:
	explicit WaitNoColorProcess(const std::shared_ptr<ColorSensor> &colorSensor);

	virtual bool isCompleted(time_t secondsFromStart) override;
	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;

	/**
	 * Установить минимальное время, в течение которого должен быть виден цвет, чтобы процесс завершился
	 * @param timeout время в секундах, значение по умолчанию 0.01 сек
	 */
	void setTimeout(time_t timeout);
protected:
	std::shared_ptr<ColorSensor> colorSensor;
	time_t lastColorTimestamp;
	bool foundNoColor;
	time_t timeout;
};

}
