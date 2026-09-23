/*
 * kalman_filter.h
 *
 *  Created on: Sep 22, 2026
 *      Author: rachel
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_


typedef struct {
    float q;
	float r;
	float x;
	float p;
	float k;
} kalman_state;

// kalman subroutine function prototype: calling convention has 2 parameters,
// 1) a struct containing the state variables, and
// 2) the current measurement which will be used to update the state
extern void kalman(kalman_state* state, float measurement);
int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);

// 4 stats functions
float* DataSubtraction(float* input, float* output, float* difference, int Length);
void CalculateMeanStd(float* array, int Length, float* mean, float* std);
void Correlation(float* A, int LengthA, float* B, int LengthB, float* corr);
void Convolution(float* A, int LengthA, float* B, int LengthB, float* convolution);

#endif /* INC_KALMAN_FILTER_H_ */
