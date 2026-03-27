/*
 * utility.h
 *
 *  Created on: Dec 21, 2023
 *      Author: VR
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_

#include "stdint.h"
#include "math.h"

//TODO make sure that is works for less than 24 bit values
int32_t sign_extend_bits_to_32(int32_t x, uint8_t bits);
double clamp_value_noref(double min_value, double value, double max_value);
void clamp_value(double *min_value, double *value, double *max_value);
float steps_to_rads(int32_t steps, int32_t full_steps);
int32_t rad_to_steps(float rads, int32_t full_steps);

#endif /* INC_UTILITY_H_ */
