/*
 * utility.c
 *
 *  Created on: Dec 21, 2023
 *      Author: VR
 */

#include "utility.h"

#include <limits.h>
#include <stddef.h>


//TODO make sure that is works for less than 24 bit values
int32_t sign_extend_bits_to_32(int32_t x, uint8_t bits) {

	uint32_t sign_mask = 0;
	//getting value of sign bit
	sign_mask = 1u << (bits - 1);
	uint32_t sign_bit = 0;
	sign_bit = x & sign_mask;
	if(sign_bit) //if value < 0 therefore sign_bit == 1, fill first 8 bits with 1
	{
		int32_t res = 0;
		int32_t mask = 0xFF;
		res |= x;
		res |= (mask << (bits));
		return res;
	}
    return x; //else return value itself
}

double clamp_value_noref(double min_value, double value, double max_value)
{
	value = (((min_value < value)? value : min_value) > max_value)? max_value: value;
	return value;
}

void clamp_value(double *min_value, double *value, double *max_value)
{
	*value = (((*min_value < *value)? *value : *min_value) > *max_value)? *max_value: *value;
}


float steps_to_rads(int32_t steps, int32_t full_steps)
{
	float rads = 0;
	rads = ((float)steps /(float)full_steps) * (M_PI * 2);
	return rads;
}

bool radians_to_steps_checked(const float radians, const int32_t full_steps, int32_t* const steps)
{
	if ((steps == NULL) || (full_steps <= 0) || !isfinite(radians)) {
		return false;
	}

	const float scaled_steps = ((float) full_steps * radians) / (2.0F * (float) M_PI);
	if (!isfinite(scaled_steps) ||
	    (scaled_steps < (float) INT32_MIN) ||
	    (scaled_steps >= 2147483648.0F)) {
		return false;
	}

	*steps = (int32_t) scaled_steps;
	return true;
}

float wrap_angle_radians(float angle)
{
	const float two_pi = 2.0F * (float) M_PI;
	float wrapped = fmodf(angle + (float) M_PI, two_pi);
	if (wrapped < 0.0F) {
		wrapped += two_pi;
	}
	return wrapped - (float) M_PI;
}

float angular_abs_diff_radians(float lhs, float rhs)
{
	return fabsf(wrap_angle_radians(lhs - rhs));
}
