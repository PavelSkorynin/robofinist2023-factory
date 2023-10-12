/*
 * WaitCrossProcess.hpp
 *
 *  Created on: 30 дек. 2019 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>
#include <Motor.h>
#include <Sensor.h>

namespace ev3 {

class WaitCrossProcess: public virtual Process {
public:
	WaitCrossProcess(MotorPtr leftMotor, MotorPtr rightMotor, SensorPtr leftLight, SensorPtr rightLight);

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	/**
	 * Установить пороговое среднее значение яркости в окне
	 * @param meanThreshold значение по умолчанию 50
	 */
	void setMeanThreshold(int meanThreshold);

protected:
	MotorPtr leftMotor;
	MotorPtr rightMotor;
	SensorPtr leftLight;
	SensorPtr rightLight;

	bool foundCross;
	int prevLeftEncoder;
	int prevRightEncoder;

	static const int WINDOW_WIDTH = 20;
	int leftLightValues[WINDOW_WIDTH];
	int rightLightValues[WINDOW_WIDTH];
	int numberOfLeftValues;
	int numberOfRightValues;

	int meanThreshold;

private:
	bool checkSensor(SensorPtr lightSensor, MotorPtr motor, int lightValues[WINDOW_WIDTH], int &prevEncoder, int &numberOfValues);
};

}
