#include "chssis.h"


float vx,vy,w;

float Wheel_Angle,Wheel_Speed;

float r,s;//车中心到轮子距离  轮子半径

float a,b;//麦克纳姆轮车体中心到轮的坐标

Steer_Wheel Steer;

All_Around_Wheel All_Around;

Mec_Wheel Mec;


//舵轮

void Steer_Calculate(void)
{
	
	Wheel_Angle = atan2f(vy+w*0.707107f*r ,vx-w*0.707107f*r);
	Wheel_Speed = sqrt((vx-w*0.707107f*r)*(vx-w*0.707107f *r)+(vy+w*0.707107f*r)*(vy+w*0.707107f*r))/ s;
	
	Steer.Angle = Wheel_Angle;
	Steer.Speed = Wheel_Speed;
	
}

//全向
void All_Around_Calculate(void)
{
  All_Around.Motor0 = (-0.707107f*vx + 0.707107f*vy + w*r) / s;
  All_Around.Motor1 = (-0.707107f*vx - 0.707107f*vy + w*r) / s;
  All_Around.Motor2 = (0.707107f*vx - 0.707107f*vy + w*r) / s;
  All_Around.Motor3 = (0.707107f*vx + 0.707107f*vy + w*r) / s;	
}

//麦克纳姆轮
void Mec_Calculate(void)
{
	Mec.Motor0 = (-vx+vy+w*(a+b))/s;
	Mec.Motor1 = (-vx-vy+w*(a+b))/s;
	Mec.Motor2 = (vx+vy+w*(a+b))/s;
	Mec.Motor3 = (vx-vy+w*(a+b))/s;
}

