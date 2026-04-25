#ifndef __CHSSIS_H
#define __CHSSIS_H

#include "struct_typedef.h"
#include "math.h"

typedef struct 
{
	float Angle;
	float Speed;
	
} Steer_Wheel;

typedef struct
{
	float Motor0;
	float Motor1;
	float Motor2;
	float Motor3;
} All_Around_Wheel;

typedef struct
{
	float Motor0;
	float Motor1;
	float Motor2;
	float Motor3;
} Mec_Wheel;

void Steer_Calculate(void);
void All_Around_Calculate(void);
	
#endif
