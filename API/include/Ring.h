/*
 * Ring.h
 *
 *  Created on: 27 июн. 2023 г.
 *      Author: Pavel
 */

#pragma once

#include <functional>

namespace ev3 {

template<typename T, int N>
class Ring {
public:
	Ring()
	: first(storage), next(storage) {
	}
	virtual ~Ring() = default;

	void push(T value) {
		*next = std::move(value);
		next++;
		if (next == end) {
			next = storage;
		}
		if (next == first) {
			first++;
			if (first == end) {
				first = storage;
			}
		}
	}

	int size() const {
		if (first == next) {
			return 0;
		}
		if (first > next) {
			return (end - first) + (next - storage);
		}
		return next - first;
	}

	void iterate(std::function<void(T&)> iteration) {
		for (T* it = first; it != next;) {
			iteration(*it);

			++it;
			if (it == end) {
				it = storage;
			}
		}
	}

	static const int capacity = N;

private:
	T storage[N+1];
	T* const end = storage + N+1;
	T* first;
	T* next;
};

} /* namespace ev3 */
