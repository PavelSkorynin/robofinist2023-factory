/*
 * Device.h
 *
 *  Created on: 10 May 2019
 *      Author: Pavel Skorynin
 */

#pragma once

#include "common.h"

namespace ev3 {

/**
 * Базовый класс для всех подключаемых блоку EV3 устройств (датчиков и моторов).
 */
	class Device {
	public:
		virtual ~Device() {}

		/**
		 * Обновление всех входных данных датчиков и энкодеров моторов.
		 * @param timestampSeconds время в секундах (нужно брать из класса EV3)
		 */
		virtual void updateInputs(time_t timestampSeconds) = 0;
		/**
		 * Обновление всех выходных данных моторов (установка мощности и скорости мотора).
		 * @param timestampSeconds время в секундах (нужно брать из класса EV3)
		 */
		virtual void updateOutputs(time_t timestampSeconds) = 0;
	};
}

