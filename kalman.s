/*
 * kalman.s
 *
 *  Created on: Sep 17, 2026
 *      Author: rachel
 */

 # Declare function as global so it can be seen by the linker
 .global kalman
 .thumb
 .syntax unified
 .thumb_func

 kalman:
 	// 0. ARM procedure call standard allows S0-S15 to be scratch registers, so do not have to push any regsiters to stack.

 	// 1. R0 contains memory address of struct with state info, S0 contains current float measured value,
 	//	  need to load all the struct variables onto registers in order to apply filter. Load is performed
 	//	  using istruction specified in Vector Floating Point Instruction Set Quick Reference Card

 	// Load S1 = q
 	VLDR S1, [R0, #0]
 	// Load S2 = r
 	VLDR S2, [R0, #4]
 	// Load S3 = x
 	VLDR S3, [R0, #8]
 	// Load S4 = p
 	VLDR S4, [R0, #12]
 	// Load S5 = k
 	VLDR S5, [R0, #16]

 	// 2. perform the actual filter operation, use floating point arithmetic:

 	//Reg P = Reg P + Reg Q
 	VADD.F32 S4, S4, S1

 	//Reg scratch = Reg P  + Reg R
 	VADD.F32 S6, S4, S2


 	// Check if divisor (S6) is zero before dividing
	VCMP.F32 S6, #0.0
	VMRS APSR_nzcv, FPSCR
	BEQ handle_zero_division

	//Reg k = Reg p / Reg scratch
	VDIV.F32 S5, S4, S6
	B continue_kalman

handle_zero_division:
	// set k to 0.0
	MOVS R1, #0
	VMOV S5, R1

continue_kalman:
 	//Reg scratch = Reg Measured - Reg X
 	VSUB.F32 S6, S0, S3

 	//Reg scratch = Reg K * Reg scratch
 	VMUL.F32 S6, S5, S6

 	//Reg X = Reg X + Reg scratch
 	VADD.F32 S3, S3, S6

 	//Reg scratch = #1 - Reg K
 	VMOV.F32 S7, #1.0
 	VSUB.F32 S6, S7, S5

 	//Reg p = Reg scratch * Reg P
 	VMUL.F32 S4, S6, S4

 	//3. STR each register back into respective mem location
 	//STR P
 	VSTR S4, [R0, #12]

 	//STR K
 	VSTR S5, [R0, #16]

 	//STR X
 	VSTR S3, [R0, #8]


 	BX LR
