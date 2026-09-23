/*
 * kalman_filter_cmsis.c
 *
 *  Created on: Sep 23, 2026
 *      Author: rachel
 *
 *  Implementation of Kalman filter using CMSIS-DSP
 */
#define ARM_MATH_CM4
#include "kalman_filter.h"
#include "arm_math.h"
#include "kalman_filter_cmsis.h"
#include <math.h>

void kalman_cmsis(kalman_state* state, float measurement){
    // p = p + q
    arm_add_f32(&state->p, &state->q, &state->p, 1);

    // scratch = p + r
    float scratch;
    arm_add_f32(&state->p, &state->r, &scratch, 1);

    // k = p / scratch
    // NOTE: CMSIS-DSP provides no floating-point division or
    // reciprocal function, so using standard C division
    state->k = state->p / scratch;

    // scratch = measurement - x
    arm_sub_f32(&measurement, &state->x, &scratch, 1);

    // scratch = k * scratch
	arm_mult_f32(&state->k, &scratch, &scratch, 1);

	// x = x + scratch
	arm_add_f32(&state->x, &scratch, &state->x, 1);

	// scratch = 1 - k  (computed directly, CMSIS doesnt have subtract from constant)
	scratch = 1.0f - state->k;

	// p = scratch * p
	arm_mult_f32(&scratch, &state->p, &state->p, 1);

}

int Kalmanfilter_cmsis(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
    for (int i = 0; i < Length; i++)
    {
        kalman_cmsis(kstate, InputArray[i]);
        if (isnan(kstate->x) || isinf(kstate->x)){
			// if there was a NaN or arithmetic error, return early
			return 1;
		}

		// assign corresponding output array spot with the new x value
		OutputArray[i] = kstate->x;
    }
    return 0;
}
