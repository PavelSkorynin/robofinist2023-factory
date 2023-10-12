/*
 * PD.h
 *
 *  Created on: 11 May 2019
 *      Author: Pavel Skorynin
 */

#pragma once

#include <functional>
#include <common.h>
#include "Wire.h"

namespace ev3 {

/**
 * Пропорционально-интегрально-дифференциальный регулятор. Используется в задаче движения по линии и
 * во многих других прикладных задачах.
 */
class PID {
public:
	/**
	 * Конструктор ПИД-регулятора
	 * @param kp коэффициент пропорциональной составляющей
	 * @param ki коэффициент интегральной составляющей
	 * @param kd коэффициент дифференциальной составляющей
	 * @param isCompleted функция, которая определяет, когда необходимо завершить процесс ПИД-регулирования
	 */
	PID(float kp = 0.4f, float ki = 0.00001f, float kd = 1.2f);

	/**
	 * Возвращает текущее значение коэффициента пропорциональности
	 */
	float getKP() const;

	/**
	 * Возвращает текущее значение интегрального коэффициента
	 */
	float getKI() const;

	/**
	 * Возвращает текущее значение дифференциального коэффициента
	 */
	float getKD() const;

	/**
	 * Установка ошибки, передаваемой в ПД-регулятор
	 * @param errorWire поток данных
	 */
	void setError(const WireF & errorWire);

	/**
	 * Возвращает текущее значение ошибки
	 */
	float getError() const;

	/**
	 * Возвращает мощность выходного компенсирующего сигнала с учётом ошибки и параметров.
	 * @return текущее воздействие
	 */
	float getPower() const;

	/**
	 * Обновление выходной мощности
	 * @param secondsFromStart текущее время в секундах
	 */
	void update(ev3::time_t secondsFromStart);


	/**
	 * Сброс накопленных значений ошибки.
	 */
	void reset();

	/**
	 * Установить коэффициенты ПИД-регулятора
	 * @param kP пропорциональная составляющая
	 * @param kI интегральная составляющая
	 * @param kD дифференциальная составляющая
	 */
	void setPID(float kP, float kI, float kD);

protected:
	float kp;
	float ki;
	float kd;
	float lastError;
	float lastIntegralPart;
	time_t lastUpdateTime;
	float power;

	WireF errorWire;
};

} /* namespace ev3 */
