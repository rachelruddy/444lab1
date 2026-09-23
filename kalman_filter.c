/*
 * kalman_filter.c
 *
 *  Created on: Sep 22, 2026
 *      Author: rachel
 *
 *  Kalman wrapper for the assembly version, calls the assembly subroutine
 */

#include "kalman_filter.h"
#include <math.h>

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
	// loop through each entry in the input array
	for (int i = 0; i < Length; i++){
		// call kalman subroutine on the ith item of the array
		kalman(kstate, InputArray[i]);

		if (isnan(kstate->x) || isinf(kstate->x)){
			// if there was a NaN or arithmetic error, return early
			return 1;
		}

		// assign corresponding output array spot with the new x value
		OutputArray[i] = kstate->x;
	}
	return 0;
}

//a
float* DataSubtraction(float* input, float* output, float* difference, int Length){
	// element-wise subtraction
    for (int i = 0; i < Length; i++){
        difference[i] = input[i] - output[i];
    }
    return difference;
}

///b
void CalculateMeanStd(float* array, int Length, float* mean, float* std){
    float sum = 0.0f;
    for (int i = 0; i < Length; i++){
        sum += array[i];
    }
    *mean = sum / (float)Length;

    sum = 0.0f;

    for (int i = 0; i < Length; i++){
        float diff = array[i] - *mean;
        sum += diff * diff;
    }
    //definition of standard deviation
    *std = sqrtf(sum / (float)Length);
}

//c
void Correlation(float* A, int LengthA, float* B, int LengthB, float* corr){
	int l = LengthA + LengthB - 1;
	// i = outer loop, which "slide" position loop is at
	for (int i = 0; i < l; i++){
		float sum = 0.0f;
		// j = walks through array B
		for (int j = 0; j < LengthB; j++){
			//idx = where A in A the B[j] pair sits, given current slide position i
			int idx = i - LengthB + 1 + j;
			if (idx >= 0 && idx < LengthA){
				sum += A[idx] * B[j];
			}
		}
		corr[i] = sum;
	}
}

//d
void Convolution(float* A, int LengthA, float* B, int LengthB, float* convolution){
    int l = LengthA + LengthB - 1;
    // i = outer loop, which "slide" position loop is at
    for (int i = 0; i < l; i++){
        float sum = 0.0f;
        // j = for a fixed slide position, goes through pairs of overlapping elements and sums their products.
        // walks through array A, and as i increases for a constant j, A[j] gets paired with
        // later elements of array B
        for (int j = 0; j < LengthA; j++){
            if ((i - j) >= 0 && (i - j) < LengthB){
                sum += A[j] * B[i - j];
            }
        }
        convolution[i] = sum;
    }
}
