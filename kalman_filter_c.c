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

void kalman_c(kalman_state* state, float measurement){
	state->p = state->p + state->q;
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement - state->x);
	state->p = (1.0f - state->k) * state->p;
}
