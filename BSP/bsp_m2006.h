#ifndef __BSP_M2006_H
#define __BSP_M2006_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx.h"
#include "can.h"

#define M2006_CAN (&hcan)

#define M2006_CNT_PER_ROUND (8191) //编码器线程
#define M2006_CNT_PER_ROUND_OUT(x) (M2006_CNT_PER_ROUND * M2006_Reduction_Ratio[(x - 1)])
#define M2006_CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define M2006_ABS(x) (x >= 0 ? x : -x)

#define M2006_LOC_DEAD_ZONE 1			 /*位置环死区*/
#define M2006_LOC_INTEGRAL_START_ERR 200 /*积分分离时对应的误差范围*/
#define M2006_LOC_INTEGRAL_MAX_VAL 8191	 /*积分范围限定，防止积分饱和*/

#define M2006_SPE_DEAD_ZONE 2			  /*速度环死区*/
#define M2006_SPE_INTEGRAL_START_ERR 2000 /*积分分离时对应的误差范围*/
#define M2006_SPE_INTEGRAL_MAX_VAL 10000  /*积分范围限定，防止积分饱和*/

typedef struct
{
	float target_val; //目标值
	float err;		  //偏差值
	float err_last;	  //上一个偏差值
	float Kp, Ki, Kd; //比例、积分、微分系数
	float integral;	  //积分值
	float output_val; //输出值
} PID;

extern uint8_t CANRxData[8];
extern uint8_t M2006_Feedback_Buf[8][7]; //电机反馈报文
extern int M2006_Pos[8];
extern uint8_t M2006_Sendbuf[8];
extern PID pid_location1, pid_location2;
extern PID pid_speed1, pid_speed2;

void M2006_Init();
void M2006_Process();
uint8_t M2006_Set_I(int target_i, uint8_t motor_id);
void M2006_Get_Feedback(uint32_t std_id, uint8_t *data_p);
int M2006_Get_Torque(uint8_t motor_id);
int M2006_Get_Speed(uint8_t motor_id);
int M2006_Get_Pos(uint8_t motor_id);
void M2006_Pos_Rec(uint8_t motor_id);
void M2006_Set_NowPos(uint8_t ID, int32_t Pos_Angle);
uint8_t M2006_Temperature(uint8_t id);
int M2006_Ang2Cnt(float angle, int ID);
double M2006_Cnt2Ang(int32_t cnt, int ID);

static void CAN_Init(CAN_HandleTypeDef *canHandle);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle);
static uint8_t CAN_SendData(CAN_HandleTypeDef *canHandle, uint8_t *pData, uint16_t ID);

static void PID_param_init(void);
static float location_pid_realize(PID *pid, float actual_val);
static float speed_pid_realize(PID *pid, float actual_val);

#endif
