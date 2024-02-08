/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:46:42
 * @LastEditTime: 2022-09-20 11:23:43
 */
#ifndef __MATH_OPS_H
#define __MATH_OPS_H

#define PI 3.14159265359f

#include <stdio.h>

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float* x, float* y, float limit);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void float32_to_float16(float* float32, unsigned short int* float16);
void float16_to_float32(unsigned short int* float16, float* float32);

#endif