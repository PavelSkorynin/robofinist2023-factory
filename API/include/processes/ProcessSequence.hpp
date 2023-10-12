/*
 * ProcessSequence.h
 *
 *  Created on: 12 сент. 2019 г.
 *      Author: Pavel Skorynin
 */

#ifndef PROCESSSEQUENCE_H_
#define PROCESSSEQUENCE_H_

#include "Process.h"
#include <queue>
#include <memory>

namespace ev3 {
class ProcessSequence: public virtual Process {
public:
	ProcessSequence();

	virtual void update(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	template<class ProcessClassA>
	ProcessSequence& operator>>=(ProcessClassA &&processA) {
		static_assert(std::is_base_of<Process, ProcessClassA>::value);
		addProcess(processA);
		return *this;
	}

	ProcessSequence& operator>>=(std::shared_ptr<Process> processA) {
		addProcess(std::move(processA));
		return *this;
	}

	template<class ProcessClass>
	void addProcess(ProcessClass &&process) {
		static_assert(std::is_base_of<Process, ProcessClass>::value);
		sequence.emplace(std::make_shared<ProcessClass>(process));
	}
	template<class ProcessClass>
	void addProcess(std::shared_ptr<ProcessClass> process) {
		static_assert(std::is_base_of<Process, ProcessClass>::value);
		sequence.emplace(std::move(process));
	}

protected:
	std::queue<std::shared_ptr<Process>> sequence;
};

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessSequence> operator>>(std::shared_ptr<ProcessClassA> processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto sequence = std::make_shared<ProcessSequence>();
	sequence->addProcess(std::move(processA));
	sequence->addProcess(std::move(processB));
	return sequence;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessSequence> operator>>(std::shared_ptr<ProcessClassA> processA, ProcessClassB &&processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto sequence = std::make_shared<ProcessSequence>();
	sequence->addProcess(std::move(processA));
	sequence->addProcess(std::move(processB));
	return sequence;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessSequence> operator>>(ProcessClassA &&processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto sequence = std::make_shared<ProcessSequence>();
	sequence->addProcess(std::move(processA));
	sequence->addProcess(std::move(processB));
	return sequence;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessSequence> operator>>(ProcessClassA &&processA, ProcessClassB &&processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto sequence = std::make_shared<ProcessSequence>();
	sequence->addProcess(processA);
	sequence->addProcess(processB);
	return sequence;
}

}
#endif /* PROCESSSEQUENCE_H_ */
