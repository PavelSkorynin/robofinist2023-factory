/*
 * ProcessGroup.h
 *
 *  Created on: 12 сент. 2019 г.
 *      Author: Pavel Skorynin
 */

#pragma once

#include "Process.h"
#include <set>
#include <vector>
#include <memory>
#include <type_traits>

namespace ev3 {

class ProcessGroupOr;
class ProcessGroupAnd;

class ProcessGroup: public virtual Process {
public:
	explicit ProcessGroup(bool completeIfAnyIsCompleted = false);

	virtual void update(time_t secondsFromStart) override;
	virtual bool isCompleted(time_t secondsFromStart) override;

	template<class ProcessClass>
	void addProcess(std::shared_ptr<ProcessClass> process) {
		static_assert(std::is_base_of<Process, ProcessClass>::value);
		group.emplace_back(std::move(process));
	}
	template<class ProcessClass>
	void addProcess(ProcessClass&& process) {
		static_assert(std::is_base_of<Process, ProcessClass>::value);
		group.emplace_back(std::make_shared<ProcessClass>(process));
	}

protected:
	std::vector<std::shared_ptr<Process>> group;
	bool completeIfAnyIsCompleted;
};

class ProcessGroupOr: public virtual ProcessGroup {
public:
	ProcessGroupOr();
	ProcessGroupOr(ProcessGroupOr&&) = default;
	ProcessGroupOr(const ProcessGroupOr&) = delete;
	virtual ~ProcessGroupOr() = default;

	ProcessGroupOr& operator=(const ProcessGroupOr&) = delete;
	ProcessGroupOr& operator=(ProcessGroupOr&&);
};

class ProcessGroupAnd: public virtual ProcessGroup {
public:
	ProcessGroupAnd();
	ProcessGroupAnd(ProcessGroupAnd&&) = default;
	ProcessGroupAnd(const ProcessGroupAnd&) = delete;
	virtual ~ProcessGroupAnd() = default;

	ProcessGroupAnd& operator=(const ProcessGroupAnd&) = delete;
	ProcessGroupAnd& operator=(ProcessGroupAnd&&);
};

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(ProcessClassA&& processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupOr>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(std::shared_ptr<Process> processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupOr>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(ProcessClassA&& processA, std::shared_ptr<Process> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupOr>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(std::shared_ptr<ProcessClassA> processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupOr>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}



template<class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(std::shared_ptr<ProcessGroupOr> processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	processA->addProcess(std::move(processB));
	return processA;
}


template<class ProcessClassA>
std::shared_ptr<ProcessGroupOr> operator|(ProcessClassA&& processA, std::shared_ptr<ProcessGroupOr> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	processB->addProcess(std::move(processA));
	return processB;
}

template<class ProcessClassB>
std::shared_ptr<ProcessGroupOr> operator|(std::shared_ptr<ProcessGroupOr> processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	processA->addProcess(std::move(processB));
	return processA;
}

template<class ProcessClassA>
std::shared_ptr<ProcessGroupOr> operator|(std::shared_ptr<ProcessClassA> processA, std::shared_ptr<ProcessGroupOr> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	processB->addProcess(std::move(processA));
	return processB;
}


template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(ProcessClassA&& processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupAnd>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(ProcessClassA&& processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupAnd>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(std::shared_ptr<ProcessClassA> processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupAnd>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}

template<class ProcessClassA, class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(std::shared_ptr<ProcessClassA> processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	auto group = std::make_shared<ProcessGroupAnd>();
	group->addProcess(std::move(processA));
	group->addProcess(std::move(processB));
	return group;
}



template<class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(std::shared_ptr<ProcessGroupAnd> processA, ProcessClassB&& processB) {
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	processA->addProcess(std::move(processB));
	return processA;
}

template<class ProcessClassB>
std::shared_ptr<ProcessGroupAnd> operator&(std::shared_ptr<ProcessGroupAnd> processA, std::shared_ptr<ProcessClassB> processB) {
	static_assert(std::is_base_of<Process, ProcessClassB>::value);
	processA->addProcess(std::move(processB));
	return processA;
}

template<class ProcessClassA>
std::shared_ptr<ProcessGroupAnd> operator&(ProcessClassA&& processA, std::shared_ptr<ProcessGroupAnd> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	processB->addProcess(std::move(processA));
	return processB;
}

template<class ProcessClassA>
std::shared_ptr<ProcessGroupAnd> operator&(std::shared_ptr<ProcessClassA> processA, std::shared_ptr<ProcessGroupAnd> processB) {
	static_assert(std::is_base_of<Process, ProcessClassA>::value);
	processB->addProcess(std::move(processA));
	return processB;
}

}

