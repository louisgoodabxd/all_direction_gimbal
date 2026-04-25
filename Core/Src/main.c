/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "chssis.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "math.h"

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.02f)	// 2 * integral gain
#define sampleFreq	1000.0f			// sample frequency in Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x) ;
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const RC_ctrl_t *RC_Ctl;



fp32 pid_motor[3]={1.9,0,0.2};
fp32 pid_motor6020_1[3]={15,0,0.3};
fp32 pid_motor6020_2[3]={10,0,0.1};

const  motor_measure_t *motor1_data;
const  motor_measure_t *motor2_data;
const  motor_measure_t *motor3_data;
const  motor_measure_t *motor4_data;
const  motor_measure_t *motor_gimbal_data;

pid_type_def motor1_pid;
pid_type_def motor2_pid;
pid_type_def motor3_pid;
pid_type_def motor4_pid;
pid_type_def motor5_1_pid;
pid_type_def motor5_2_pid;

float wheel_vx,wheel_vy,wheel_w;
float gimbal_vx,gimbal_vy,gimbal_w;
float  Motor1_speed, Motor2_speed, Motor3_speed, Motor4_speed;
float wheel_r,wheel_s;

//IMU
volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
fp32 yaw;
fp32 pitch; 
fp32 roll;
fp32 gyro[3], accel[3], temp;
fp32 mag[3];
fp32 INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.????? ??�� rad
// yaw setpoint for incremental control (遥控通道1增量控制)
fp32 yaw_setpoint = 0.0f;
uint8_t yaw_setpoint_init = 0;
const fp32 RC_CH_SCALE = 660.0f; // RC 通道归一化因子（根据遥控器范围调整）
const fp32 MAX_YAW_RATE = 1.0471975512f; // 最大偏航速度，rad/s (60 deg/s)
const int RC_DEADBAND = 20; // 遥控死区

typedef struct 
{
	fp32 GX;
	fp32 GY;
	fp32 GZ;
} GYRO;
typedef struct 
{
	fp32 AX;
	fp32 AY;
	fp32 AZ;
	fp32 temperature;
} ACCEL;
typedef struct
{
	fp32 MAG_X;
	fp32 MAG_Y;
	fp32 MAG_Z;
} IST8310_DATA;

GYRO GYRO1;
ACCEL ACCEL1;
IST8310_DATA MAG1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        ist8310_read_mag(mag);
    }

}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  remote_control_init(); 
  RC_Ctl=get_remote_control_point();
	can_filter_init();
	
	//IMU
	ist8310_init();
	while(BMI088_init())
    {
        ;
    }
	
	motor1_data = get_chassis_motor_measure_point(0);
	motor2_data = get_chassis_motor_measure_point(1);
	motor3_data = get_chassis_motor_measure_point(2);
	motor4_data = get_chassis_motor_measure_point(3);
	motor_gimbal_data = get_chassis_motor_measure_point(8);
//	
	PID_init(&motor1_pid ,PID_POSITION ,pid_motor ,10000, 2000);
	PID_init(&motor2_pid ,PID_POSITION ,pid_motor ,10000, 2000);
	PID_init(&motor3_pid, PID_POSITION, pid_motor, 10000, 2000);
	PID_init(&motor4_pid, PID_POSITION, pid_motor, 10000, 2000);
	PID_init(&motor5_1_pid, PID_POSITION, pid_motor6020_1, 300, 50);
	PID_init(&motor5_2_pid, PID_POSITION, pid_motor6020_2, 16000, 2000);
		
//	

//	
	wheel_r = 0.19;
	wheel_s = 0.075;
		

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//IMU
		BMI088_read(gyro, accel, &temp);
	  AHRS_update(INS_quat, 0.001f, gyro, accel, mag);
	  get_angle(INS_quat, &yaw, &pitch, &roll);
  
		gimbal_vx = -RC_Ctl->rc .ch[2];
		gimbal_vy = RC_Ctl->rc .ch[3];
		gimbal_w = RC_Ctl->rc .ch[0]*2;

		
		wheel_vx = gimbal_vx * cosf(-yaw) - gimbal_vy * sinf(-yaw);
    wheel_vy = gimbal_vx * sinf(-yaw) + gimbal_vy * cosf(-yaw);
    wheel_w = gimbal_w;
//		
		 Motor1_speed = (-0.707107f*wheel_vx + 0.707107f*wheel_vy + wheel_w*wheel_r) / wheel_s;
		 Motor2_speed = (-0.707107f*wheel_vx - 0.707107f*wheel_vy + wheel_w*wheel_r) / wheel_s;
		 Motor3_speed = (0.707107f*wheel_vx - 0.707107f*wheel_vy + wheel_w*wheel_r) / wheel_s;
		 Motor4_speed = (0.707107f*wheel_vx + 0.707107f*wheel_vy + wheel_w*wheel_r) / wheel_s;	
//		
		 PID_calc(&motor1_pid ,motor1_data->speed_rpm, Motor1_speed);
		 PID_calc(&motor2_pid ,motor2_data->speed_rpm, Motor2_speed);
		 PID_calc(&motor3_pid ,motor3_data->speed_rpm, Motor3_speed);
		 PID_calc(&motor4_pid ,motor4_data->speed_rpm, Motor4_speed);
		// 初始化 yaw_setpoint 为当前陀螺角度（只在第一次循环）
		if(!yaw_setpoint_init)
		{
			yaw_setpoint = yaw;
			yaw_setpoint_init = 1;
		}

		// 通道1作为增量式控制：根据通道值产生角速度指令，再积分得到目标角度
		{
			int16_t ch1 = RC_Ctl->rc.ch[1];
			if(ch1 > -RC_DEADBAND && ch1 < RC_DEADBAND)
			{
				// 小幅度输入视为0
			}
			else
			{
				// 将遥控量映射为角速度（rad/s），然后积分得到角度增量
				fp32 yaw_rate_cmd = ((fp32)ch1) / RC_CH_SCALE * MAX_YAW_RATE;
				const fp32 dt = 0.001f; // 主循环周期，s（与 AHRS_update 中时间一致）
				yaw_setpoint += yaw_rate_cmd * dt;
				// 归一化到 [-PI, PI]
				if(yaw_setpoint > M_PI) yaw_setpoint -= 2.0f * M_PI;
				if(yaw_setpoint < -M_PI) yaw_setpoint += 2.0f * M_PI;
			}
		}

		PID_calc(&motor5_1_pid, yaw, yaw_setpoint);
		PID_calc(&motor5_2_pid ,motor_gimbal_data->speed_rpm , motor5_1_pid.out );
		 
		// PID_calc(&motor5_pid ,motor_gimbal_data->speed_rpm , motor5_pid.out);
//		
		 CAN_cmd_chassis(motor1_pid.out,motor2_pid.out ,motor3_pid.out,motor4_pid.out);
		 //CAN_cmd_gimbal(motor5_2_pid.out ,0,0,0);
		 CAN_cmd_gimbal(motor5_2_pid.out ,0,0,0);
		
		 		HAL_Delay (1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void  AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])                             
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
