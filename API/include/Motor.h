/*
 * Motor.h
 *
 *  Created on: 09 May 2019
 *      Author: Pavel Skorynin
 */

#pragma once

#include "core/ev3_constants.h"
#include "Device.h"
#include "Wire.h"

#include <memory>

namespace ev3 {

/**
 * Класс, отвечающий за взаимодействие с мотором, подключенным к блоку EV3.
 */
class Motor : public Device {
public:
	/**
	 * Порт подключения мотора
	 */
	enum class Port : uint8_t {
		A = OUT_A,//!< A
		B = OUT_B,//!< B
		C = OUT_C,//!< C
		D = OUT_D //!< D
	};

	/**
	 * Направление вращения мотора.
	 * Если мотор вращается в обратную сторону, getActualSpeed и getEncoder будут возвращать значения,
	 * соответствующие этому направлению.
	 */
	enum class Direction {
		FORWARD = OUT_FWD,//!< FORWARD
		BACKWARD = OUT_REV//!< BACKWARD
	};

	~Motor();

	/**
	 * Порт, к которому подключен мотор
	 * @return порт
	 */
	inline Port getPort() const {
		return port;
	}

	/**
	 * Направление вращения мотора
	 * @return направление
	 */
	inline Direction getDirection() const {
		return direction;
	}
	/**
	 * Устанавливает направление вращения мотора
	 * @param direction направление
	 */
	void setDirection(const Direction & direction);

	/**
	 * Инвертирует направление движения. Был FORWARD - стал BACKWARD и наоборот
	 */
	void toggleDirection();

	/**
	 * Текущая скорость вращения мотора.
	 * Положительная, если фактическое направление вращения мотора совпадает с выбранным.
	 * @return Скорость в интервале [0, 100]
	 */
	int getActualSpeed() const;

	/**
	 * Текущая скорость вращения мотора.
	 * Положительная, если фактическое направление вращения мотора совпадает с выбранным.
	 * @return провод с данными текущей скорости
	 */
	WireI getActualSpeedWire() const;

	/**
	 * Текущее значение енкодера.
	 * Положительное, если фактическое направление вращения мотора совпадает с выбранным.
	 * @return значение в градусах
	 */
	int getEncoder() const;
	/**
	 * Текущее значение енкодера.
	 * Положительное, если фактическое направление вращения мотора совпадает с выбранным.
	 * @return провод с данными текущего значения енкодера
	 */
	WireI getEncoderWire() const;

	/**
	 * Текущее значение енкодера, без учёта выбранного направления.
	 * @return значение в градусах
	 */
	int getTacho() const;
	/**
	 * Текущее значение енкодера, без учёта выбранного направления.
	 * @return провод с данными текущего значения енкодера без учёта направления
	 */
	WireI getTachoWire() const;

	/**
	 * Установка мощности мотора. Нельзя задавать одновременнно с setPower.
	 * Будет работать только последний вызов.
	 * @param power требуемая мощность
	 */
	void setPower(int power);
	/**
	 * Установка мощности мотора. Нельзя задавать одновременнно с setPower.
	 * Будет работать только последний вызов.
	 * @param output поток данных
	 */
	void setPower(const WireI & output);

	/**
	 * Актуальная мощность мотора, установленная на последней итерации
	 * @return мощность мотора в интервале [0, 100]
	 */
	float getActualPower() const;

	/**
	 * Установленная через setPower мощность мотора. Может отличаться от актуальной мощности,
	 * если мотор ещё не раскрутился.
	 * @return мощность мотора в интервале [0, 100]
	 */
	int getPower() const;

	/**
	 * Сбрасывает значение энкодера
	 */
	void resetEncoder();

	/**
	 * Блокирует мотор на указанном значении енкодера
	 * @param targetEncoder целевое значение енкодера
	 */
	void blockOnEncoder(int targetEncoder);

	/**
	 * Максимальное ускорение мотора в градусах на секунду в квадрате. По умолчанию 2500.
	 * Допустимые значения лежат в интервале примерно [1000, 10000]
	 */
	void setMaxAccelleration(float maxAcceleration);

	/**
	 * Масштабирование показаний енкодера. Используется для калибровки моторов.
	 * При вызове getEncoder исходное значение будет умножаться на scale.
	 * @param scale масштаб
	 */
	void setEncoderScale(float scale);

	/**
	 * Установка скорости на максимальной мощности (100).
	 * Используется для калибровки моторов и корректного рассчёта ускорения.
	 * По умолчанию используется значение 850 - большой мотор с небольшой нагрузкой
	 * @param degreesPerSecond скорость в градусах за секунду
	 */
	void setSpeedOnMaxPower(float degreesPerSecond);

	/**
	 * Минимальная мощность, необходимая для старта.
	 * Используется для преодоления силы трения на старте.
	 * По умолчанию используется значение 3
	 * @param startUpPower мощность в интервале [0, 100]
	 */
	void setStartUpPower(int startUpPower);

	/**
	 * Обновляет значение энкодера
	 * @param timestampSeconds текущее время в секундах
	 */
	void updateInputs(time_t timestampSeconds) override;
	/**
	 * Обновляет выходную мощность или скорость мотора с учётом максимального ускорения.
	 * @param timestampSeconds текущее время в секундах
	 */
	void updateOutputs(time_t timestampSeconds) override;

	inline float getMaxAccelleration() const { return maxAcceleration; }

	inline float getEncoderScale() const { return encoderScale; }

	inline float getSpeedOnMaxPower() const { return powerToSpeedRatio * 100; }

	inline int getStartUpPower() const { return startUpPower; }

	/**
	 * Проверяет, занят ли мотор. Мотор может быть занят, когда выполняются продолжительные операции.
	 * Например, OutputStepPowerEx, OutputStepSyncEx и пр.
	 */
	bool isBusy();

protected:
	Port port;
	Direction direction = Direction::FORWARD;
	float finePower = 0;
	time_t prevTimestamp = 0;

	int8_t actualSpeed = 0;
	int zeroEncoder = 0;
	int encoder = 0;
	int tacho = 0;

	float maxAcceleration = 2500;
	float encoderScale = 1;
	float powerToSpeedRatio = 8.5f;
	int startUpPower = 3;

	WireI speedInput;
	WireI encoderInput;
	WireI tachoInput;
	std::shared_ptr<WireI> powerOutput;
	std::shared_ptr<WireI> speedOutput;

	Motor(Port port);

	friend class EV3;
};


typedef std::shared_ptr<Motor> MotorPtr;

} /* namespace ev3 */
