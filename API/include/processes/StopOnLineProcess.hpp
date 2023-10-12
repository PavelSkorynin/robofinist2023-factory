/*
 * StopOnLineProcess.h
 *
 *  Created on: 27 дек. 2019 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>
#include <Motor.h> // для моторов
#include <Sensor.h> // для датчиков
#include <PID.h>
#include "MoveOnLineProcess.hpp"

namespace ev3 {

class StopOnLineProcess: public virtual Process {
public:
	StopOnLineProcess(MotorPtr leftMotor, MotorPtr rightMotor,
			SensorPtr leftLight, SensorPtr rightLight,
			int encoderDistance, int maxPower = 70, std::shared_ptr<PID> movePID = nullptr, std::shared_ptr<PID> powerPID = nullptr);

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;
	virtual void onCompleted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	/**
	 * Установить ПИД-регулятор для движения по энкодеру
	 * Значения по умолчанию: 0.3f, 0, 0.9f
	 */
	void setMovePID(std::shared_ptr<PID> pid);

	/**
	 * Установить ПИД-регулятор для максимальной мощности
	 * Значения по умолчанию: 0.5f, 0.0f, 1.2f
	 */
	void setPowerPID(std::shared_ptr<PID> pid);

	/**
	 * Установить минимальное значение устанавливаемой скорости.
	 * Мощность на моторы будет подаваться не меньше этого значения
	 * @param minPower минимальная подаваемая мощность, значение по умолчанию 7
	 */
	void setMinPower(int minPower);

	/**
	 * Установить значение енкодера при котором начинается резкое замедление
	 * @param anchorEncoder переломное значение енкодера, значение по умолчанию 50
	 */
	void setAnchorEncoder(float anchorEncoder);

	/**
	 * Мощность, подаваемая на моторы, при которой завершается процесс
	 * @param powerThreshold минимальная мощность, значение по умолчанию 5
	 */
	void setPowerThreshold(int powerThreshold);

	/**
	 * Скорость на моторах (actual speed), при которой завершается процесс
	 * @param speedThreshold минимальная скорость, значение по умолчанию 3
	 */
	void setSpeedThreshold(int speedThreshold);

	/**
	 * Допустимое отклонение от требуемого значения енкодера в градусах, при котором завершается процесс
	 * @param distanceThreshold минимальная отклонение, значение по умолчанию 5
	 */
	void setDistanceThreshold(int distanceThreshold);

protected:
	MotorPtr leftMotor;
	MotorPtr rightMotor;
	SensorPtr leftLight;
	SensorPtr rightLight;
	int encoderDistance;
	int maxPower;
	int minPower;
	float anchorEncoder;
	int powerThreshold;
	int speedThreshold;
	int distanceThreshold;

	float encoderStart;

	std::shared_ptr<PID> powerPD;
	std::shared_ptr<MoveOnLineProcess> moveOnLineProcess;
};

}
