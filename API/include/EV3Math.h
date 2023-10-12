/*
 * EV3Math.h
 *
 *  Created on: 7 окт. 2019 г.
 *      Author: Pavel
 */

#pragma once

namespace ev3 {

	template<typename T>
	inline T clamp(T value, T min, T max) {
		return value < min ? min : ((value > max) ? max : value);
	}

	template<typename T>
	inline T map(T value, T minSrc, T maxSrc, T minDst, T maxDst) {
		return maxSrc <= minSrc ? minDst : (value - minSrc) * (maxDst - minDst) / (maxSrc - minSrc) + minDst;
	}

	template<typename T>
	inline T min(T value1, T value2) {
		return value1 < value2 ? value1 : value2;
	}

	template<typename T>
	inline T max(T value1, T value2) {
		return value1 < value2 ? value2 : value1;
	}

}
