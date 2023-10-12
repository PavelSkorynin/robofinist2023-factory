//============================================================================
// Name        : robofinist2023.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : EV3 WRO C++ template
//============================================================================

#include <ev3.h>
#include <processes.h>

using namespace ev3;
using namespace std;

const int RED_COLOR = 0;
const int YELLOW_COLOR = 1;
const int GREEN_COLOR = 2;
const int BLUE_COLOR = 3;

vector<int> colors(4, 0);

int main()
{
	auto ev3 = new EV3();
	const int power = 100;

	// инициализация устройств
	ev3->initSensors(Sensor::Mode::COLOR_REFLECT, Sensor::Mode::COLOR_REFLECT, Sensor::Mode::COLOR_RGB);
	auto leftLight = ev3->getReflectedLightSensor(Sensor::Port::P1);
	auto rightLight = ev3->getReflectedLightSensor(Sensor::Port::P2);
	auto colorSensor = ev3->getColorSensor(Sensor::Port::P3);
	auto leftMotor = ev3->getMotor(Motor::Port::A);
	auto rightMotor = ev3->getMotor(Motor::Port::B);

	// сброс моторов
	leftMotor->setPower(0);
	rightMotor->setPower(0);

	// калибровка
	leftLight->setMinValue(9);
	leftLight->setMaxValue(91);
	rightLight->setMinValue(11);
	rightLight->setMaxValue(93);
	colorSensor->setMaxRValue(100);
	colorSensor->setMaxGValue(100);
	colorSensor->setMaxBValue(100);
	colors[RED_COLOR] = 0;
	colors[YELLOW_COLOR] = 40;
	colors[GREEN_COLOR] = 130;
	colors[BLUE_COLOR] = 240;
	leftMotor->setDirection(Motor::Direction::FORWARD);
	rightMotor->setDirection(Motor::Direction::FORWARD);
	leftMotor->setEncoderScale(1);
	rightMotor->setEncoderScale(1);
	leftMotor->setMaxAccelleration(4000);
	rightMotor->setMaxAccelleration(4000);

	// движемся по линии, пока не встретим цвет на датчике цвета
	ev3->runProcess(MoveOnLineProcess(leftMotor, rightMotor, leftLight, rightLight, 10000, power)
			& WaitColorProcess(colorSensor));
	// проигрываем звук
	ev3->playSound(300, 0.1f, 0.05f);
	// останавливаем правый и левый моторы
	ev3->runProcess(StopProcess(leftMotor) | StopProcess(rightMotor));

	ev3->wait(1);
	delete ev3;
	return 0;
}
