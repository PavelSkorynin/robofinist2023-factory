/*
 * MoveOnLineProcess.hpp
 *
 *  Created on: 27 дек. 2019 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>
#include <Motor.h> // для моторов
#include <Sensor.h> // для датчиков
#include <PID.h>

namespace ev3 {

class MoveOnLineProcess: public virtual Process {
public:
	MoveOnLineProcess(MotorPtr leftMotor, MotorPtr rightMotor,
			SensorPtr leftLight, SensorPtr rightLight,
			int encoderDistance, int maxPower = 70, std::shared_ptr<PID> pid = nullptr);

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;
	virtual void onCompleted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	/**
	 * Установить максимальную мощность двигателей
	 * @param maxPower максимальная мощность (от 0 до 100)
	 */
	void setMaxPower(int maxPower);

	/**
	 * Установить коэффициенты ПИД-регулятора
	 * Значения по умолчанию: 0.3f, 0, 0.9f
	 */
	void setPID(std::shared_ptr<PID> pid);

protected:
	MotorPtr leftMotor;
	MotorPtr rightMotor;
	SensorPtr leftLight;
	SensorPtr rightLight;
	int encoderDistance;
	int leftEncoderStart = 0;
	int rightEncoderStart = 0;
	int maxPower;

	std::shared_ptr<PID> pd;
};

}
