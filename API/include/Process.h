/*
 * Process.h
 *
 *  Created on: 11 May 2019
 *      Author: Pavel Skorynin
 */

#pragma once

#include "common.h"

#include <functional>

namespace ev3 {
/**
 * Класс для организации процесса обработки данных
 */
	class Process {
	protected:
		bool isStarted;
	public:
		Process() : isStarted(false) {}
		virtual ~Process() = default;

		/**
		 * Обновление
		 * @param secondsFromStart текущее время в секундах
		 */
		virtual void update(time_t secondsFromStart);

		/**
		 * Метод вызывается, когда первый раз вызывается update
		 * @param secondsFromStart
		 */
		virtual void onStarted(time_t secondsFromStart);

		/**
		 * Метод вызывается, когда процесс завершён
		 * @param secondsFromStart
		 */
		virtual void onCompleted(time_t secondsFromStart);

		/**
		 * Признак завершённости процесса
		 * @return true, если процесс завершён и больше вызывать update не имеет смысла
		 */
		virtual bool isCompleted(time_t secondsFromStart);
	};

	class LambdaProcess : public virtual Process {
	public:
		LambdaProcess(const std::function<bool(time_t)> &updateFunc);
		LambdaProcess(const std::function<bool(time_t)> &updateFunc, const std::function<void(time_t)> &onCompletedFunc);

		virtual void update(time_t secondsFromStart) override;
		virtual void onCompleted(time_t secondsFromStart) override;
		virtual bool isCompleted(time_t secondsFromStart) override;

	protected:
		std::function<bool(time_t)> updateFunc;
		std::function<void(time_t)> onCompletedFunc;
		bool completed;
	};

	class TimeProcess : public virtual Process {
	public:
		TimeProcess(const std::function<void(time_t)> &updateFunc, time_t duration, time_t delay = 0.0f);
		TimeProcess(const std::function<void(time_t)> &updateFunc, const std::function<void(time_t)> &onCompletedFunc, time_t duration, time_t delay = 0.0f);

		virtual void onStarted(time_t secondsFromStart) override;
		virtual void update(time_t secondsFromStart) override;
		virtual void onCompleted(time_t secondsFromStart) override;
		virtual bool isCompleted(time_t secondsFromStart) override;

	protected:
		std::function<void(time_t)> updateFunc;
		std::function<void(time_t)> onCompletedFunc;
		bool completed;
		time_t startTime;
		time_t duration;
		time_t delay;
	};
}
