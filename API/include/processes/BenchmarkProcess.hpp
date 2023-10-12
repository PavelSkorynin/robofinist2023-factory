/*
 * BenchmarkProcess.hpp
 *
 *  Created on: 24 янв. 2020 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include <Process.h>

namespace ev3 {

class BenchmarkProcess: public virtual Process {
public:
	BenchmarkProcess();
	virtual ~BenchmarkProcess() = default;

	virtual void update(time_t secondsFromStart) override;
	virtual void onStarted(time_t secondsFromStart) override;

	time_t getStartTime() const { return startTime; }
	time_t getCompleteTime() const { return completeTime; }
	float getDuration() const { return completeTime - startTime; }
	int getNumberOfIterations() const { return numberOfIterations; }
	float getNumberOfIterationsPerSecond() const {
		time_t duration = getDuration();
		if (duration > 0) {
			return numberOfIterations / duration;
		}
		return 0;
	}

protected:
	time_t startTime = 0;
	time_t completeTime = 0;
	int numberOfIterations = 0;
};

} /* namespace ev3 */

