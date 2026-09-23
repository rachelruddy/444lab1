/*
 * kalman_filter_c.c
 *
 *  Created on: Sep 23, 2026
 *      Author: rachel
 *
 *  Complete C Kalman implementation
 */

//include for the kalman_state struct
#include "kalman_filter_c.h"
#include "kalman_filter.h"
#include <math.h>

void kalman_c(kalman_state* state, float measurement){
	state->p = state->p + state->q;
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement - state->x);
	state->p = (1.0f - state->k) * state->p;
}

int Kalmanfilter_c(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
    for (int i = 0; i < Length; i++)
    {
        kalman_c(kstate, InputArray[i]);
        if (isnan(kstate->x) || isinf(kstate->x)){
			// if there was a NaN or arithmetic error, return early
			return 1;
		}

		// assign corresponding output array spot with the new x value
		OutputArray[i] = kstate->x;
    }
    return 0;
}
