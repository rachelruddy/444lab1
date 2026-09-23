/*
 * kalman_filter_cmsis.h
 *
 *  Created on: Sep 23, 2026
 *      Author: rachel
 */

#ifndef INC_KALMAN_FILTER_CMSIS_H_
#define INC_KALMAN_FILTER_CMSIS_H_

#include "kalman_filter.h"

void kalman_cmsis(kalman_state* state, float measurement);
int Kalmanfilter_cmsis(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);


#endif /* INC_KALMAN_FILTER_CMSIS_H_ */
