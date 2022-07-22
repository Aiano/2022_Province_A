/** 
* @brief    RoboMaster M2006无刷电机板级支持包
* @details  用于RoboMaster M2006无刷电机通过CAN协议控制
* @author   HGF
* @date     2021.11
* @version  1.1
* @par Copyright (c):  HGF
* @par 日志
*			V1.0 移植自One Point Five前辈代码
*			V1.1 简单合并文件，整理代码
*
*/
#include "bsp_m2006.h"

uint8_t CANRxData[8] = {0};
uint8_t M2006_Feedback_Buf[8][7]; //电机反馈值(全局变量)
uint8_t M2006_Sendbuf[8] = {0};      //CAN发送数据
int M2006_Pos[8];                  //每一个元素对应一个ID的电机的信息

PID pid_location1, pid_location2; //定义位置PID与速度PID结构体
PID pid_speed[4];

/**
  * @name   M2006_Init
  * @brief  M2006电机初始化
  * @param  None
  * @retval None
  */
void M2006_Init() {
    PID_param_init();
    CAN_Init(M2006_CAN);
//	HAL_CAN_Start(M2006_CAN);
}

/**
  * @name   M2006_Process
  * @brief  M2006电机控制程序
  * @param  None
  * @retval None
  */
void M2006_Process(uint8_t motor_id) {
    int GetSpeed;

    /************* 双环 *************/
    //		RM3508_Pos_Rec(1);
    //		pid_speed1.target_val =	location_pid_realize(&pid_location1,RM3508_Get_Pos(1));
    //		RM3508_Get_Feedback(0x201,CANRxData);
    //		GetSpeed=RM3508_Get_Speed(1);
    //		RM3508_Set_I( speed_pid_realize(&pid_speed1,GetSpeed), 1);

    /************* 单环 *************/


    GetSpeed = M2006_Get_Speed(motor_id);

    M2006_Set_I(speed_pid_realize(pid_speed + motor_id - 1, GetSpeed), motor_id); // 电流设置

}

/**
  * @name   M2006_Set_I
  * @brief  M2006电机电流设置
  * @param  target_i 目标电流
  * @param  motor_id 电机id
  * @retval 成功返回0，失败返回1
  */
uint8_t M2006_Set_I(int target_i, uint8_t motor_id) {
    CAN_TxHeaderTypeDef TxMessage;

    if (motor_id >= 1 && motor_id <= 8) { // 最多同时操纵8个电机

        if (target_i <= -10000) // 限制目标电流值的范围 -10000 ~ 10000
            target_i = -10000;
        else if (target_i >= 10000)
            target_i = 10000;

        int send_id = 0; // 设置发送报文的帧ID
        if (motor_id <= 4) // 根据C610数据手册，编号为1 ~ 4的电机发送报文ID为0x200, 5 ~ 8的为0x1FF
            send_id = 0x200;
        else {
            send_id = 0x1ff;
            motor_id -= 4;
        }

        // 只修改对应电机的位，其它位保持不变
        M2006_Sendbuf[2 * motor_id - 2] = target_i >> 8;     //电流值高8位
        M2006_Sendbuf[2 * motor_id - 1] = target_i & 0x00ff; //电流值低8位

        CAN_SendData(M2006_CAN, M2006_Sendbuf, send_id);

        return 0;
    } else
        return 1;
}

/**
  * @name   M2006_Get_Feedback
  * @brief  获取M2006电机的反馈并存入M2006_Feedback_Buf
  * @param  std_id message_id
  * @param  data_p message数组指针
  * @retval None
  */
void M2006_Get_Feedback(uint8_t *data_p) {
    int i;
    for (i = 1; i <= 4; i++) //前四电机匹配
    {
//		if (std_id == 0x200 + i)
//		{
        memcpy(M2006_Feedback_Buf[i - 1], data_p, 7);
//        M2006_Pos_Rec(i);

//		}
    }
    return;
}

/**
  * @name   M2006_Get_Real_I
  * @brief  获取M2006电机的实际转矩信息
  * @param  motor_id 电机id号
  * @retval 对应id电机的转矩,读取失败返回0
  */
int M2006_Get_Torque(uint8_t motor_id) {
    int torque = 0;
    if (M2006_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        torque = -(0xffff - ((M2006_Feedback_Buf[motor_id - 1][4] << 8) + M2006_Feedback_Buf[motor_id - 1][5]));
    else
        torque = (M2006_Feedback_Buf[motor_id - 1][4] << 8) + M2006_Feedback_Buf[motor_id - 1][5];
    return torque;
}

/**
  * @name   M2006_Get_Speed
  * @brief  获取M2006电机的反馈的速度信息
  * @param  motor_id 电机id号
  * @retval 对应id电机的速度,读取失败返回0
  */
int M2006_Get_Speed(uint8_t motor_id) {
    int speed = 0;
    if (M2006_Feedback_Buf[motor_id - 1][2] >> 7 == 1)
        speed = -(0xffff - ((M2006_Feedback_Buf[motor_id - 1][2] << 8) + M2006_Feedback_Buf[motor_id - 1][3]));
    else
        speed = (M2006_Feedback_Buf[motor_id - 1][2] << 8) + M2006_Feedback_Buf[motor_id - 1][3];
    return speed;
}

/**
  * @name   M2006_Get_Pos
  * @brief  获取M2006电机当前的位置信息
  * @param  motor_id 电机id号
  * @retval 对应id电机的位置,读取失败返回0
  */
int M2006_Get_Pos(uint8_t motor_id) {
    return M2006_Pos[motor_id - 1];
}

/**
  * @name   M2006_Pos_Rec
  * @brief  获取M2006电机的反馈的位置信息
  * @param  motor_id 电机id号
  * @retval 对应id电机的位置信息,读取失败返回-1
  */
static int32_t M2006_base[8] = {0}; //用来标记已经转过的圈数，一圈8192

void M2006_Pos_Rec(uint8_t motor_id) {
    int id = motor_id - 1;
    static int32_t M2006_tmp[8];
    static int32_t M2006tmp_pre[8] = {0};

    M2006_tmp[id] = (M2006_Feedback_Buf[id][0] << 8) + M2006_Feedback_Buf[id][1];
    if (M2006_tmp[id] - M2006tmp_pre[id] > 4095) //转过8191到0时记录圈数
        M2006_base[id] -= 8191;
    else if (M2006_tmp[id] - M2006tmp_pre[id] < -4095)
        M2006_base[id] += 8191;

    M2006tmp_pre[id] = M2006_tmp[id];
    M2006_Pos[id] = M2006_base[id] + M2006_tmp[id];
}

/**
  * @name   M2006_Ang2Cnt
  * @brief  角度转换为实际电机应该转动的位置数
  * @param  angle 目标角度（任意角度）
  * @param  ID 电机id号
  * @retval 电机位置
  */
//int M2006_Ang2Cnt(float angle, int ID)
//{
//	int cnt;
//	cnt = (int)(M2006_CNT_PER_ROUND_OUT(ID) * angle / 360);
//	return cnt;
//}

/**
  * @name   M2006_Cnt2Ang
  * @brief  电机位置转换为角度
  * @param  cnt 电机位置
  * @param  ID 电机id号
  * @retval 电机转过的角度
  */
//double M2006_Cnt2Ang(int32_t cnt, int ID)
//{
//	double angled;
//	angled = (double)((cnt * 360.0) / M2006_CNT_PER_ROUND_OUT(ID));
//	return angled;
//}

/**
  * @name   M2006_Set_NowPos
  * @brief  将2006电机的当前值设置为任意位置
  * @param  ID 电机id号
  * @param  Pos_Angle 目标电机位置
  * @retval None
  */
void M2006_Set_NowPos(uint8_t ID, int32_t Pos_Angle) {
    uint8_t id;
    id = ID - 1;
    M2006_base[id] = Pos_Angle;
}

/**
  * @name   M2006_Set_NowPos
  * @brief  将2006电机的当前值设置为零点
  * @param  ID 电机id号
  * @param  Pos_Angle 当前电机位置
  * @retval None
  */
void M2006_Set_ZeroPos(uint8_t ID, float Pos_Angle) {
}

uint8_t M2006_Temperature(uint8_t id) {
    uint8_t Tem;
    Tem = M2006_Feedback_Buf[id - 1][6];
    return Tem;
}

/**
  * @name   CAN_Init
  * @brief  初始化CAN及过滤器
  * @param  canHandle CAN配置结构体
  * @retval None
  */
static void CAN_Init(CAN_HandleTypeDef *canHandle) {
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;

    if (M2006_CAN->Instance == canHandle->Instance) {
        sFilterConfig.FilterBank = 0;                      //过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; //filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(M2006_CAN, &sFilterConfig);
        HAL_CAN_Start(M2006_CAN); //开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(M2006_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    if (HAL_Status != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @name   HAL_CAN_RxFifo0MsgPendingCallback
  * @brief  CAN FIFO0回调
  * @param  canHandle CAN配置结构体
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle) {
    // 一次接收中断只处理一个电机的数据
    CAN_RxHeaderTypeDef RxMessage; // 接收帧头结构体

    RxMessage.StdId = 0;
    RxMessage.ExtId = 0;
    RxMessage.IDE = CAN_ID_STD;
    RxMessage.RTR = CAN_RTR_DATA;
    RxMessage.DLC = 8; // 数据长度8Byte
    RxMessage.Timestamp = 0;
    RxMessage.FilterMatchIndex = 0;

    HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &RxMessage, CANRxData); // CANRxData: 接收数据

    int index;
    index = RxMessage.StdId - 0x200;
    memcpy(M2006_Feedback_Buf[index - 1], CANRxData, 7); // 将对应电机的反馈数据存到对应缓冲区

    M2006_Process(index); // 根据反馈数据进一步处理，一次中断只处理一个电机
}

/**
  * @name   CAN_SendData
  * @brief  CAN发送函数
  * @param  canHandle CAN配置结构体
  * @param  pData message数组指针
  * @param  ID message_id
  * @retval None
  */
static uint8_t CAN_SendData(CAN_HandleTypeDef *canHandle, uint8_t *pData, uint16_t ID) {
    HAL_StatusTypeDef HAL_RetVal = HAL_ERROR;
    uint8_t FreeTxNum = 0;
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.StdId = ID;
    TxMessage.DLC = 8; /*默认一帧传输长度为8*/
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;

    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(canHandle);

    while (FreeTxNum == 0) //等待空邮箱，可能会卡死在这里（小BUG）
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(canHandle);
    }

    HAL_RetVal = HAL_CAN_AddTxMessage(canHandle, &TxMessage, pData, (uint32_t *) CAN_TX_MAILBOX1);

    if (HAL_RetVal != HAL_OK) {
        return 2;
    }

    return 0;
}

/**
  * @name   PID_param_init
  * @brief  PID参数初始化
  * @param  None
  * @retval None
  */
static void PID_param_init() {
    /* 速度相关初始化参数 */
    pid_speed[0].target_val = 3000.0;
    pid_speed[0].output_val = 0.0;
    pid_speed[0].err = 0.0;
    pid_speed[0].err_last = 0.0;
    pid_speed[0].integral = 0.0;
    pid_speed[0].Kp = 20;
    pid_speed[0].Ki = 0.0;
    pid_speed[0].Kd = 0.0;

    pid_speed[1].target_val = -3000.0;
    pid_speed[1].output_val = 0.0;
    pid_speed[1].err = 0.0;
    pid_speed[1].err_last = 0.0;
    pid_speed[1].integral = 0.0;
    pid_speed[1].Kp = 20;
    pid_speed[1].Ki = 0.0;
    pid_speed[1].Kd = 0.0;

    pid_speed[2].target_val = 3000.0;
    pid_speed[2].output_val = 0.0;
    pid_speed[2].err = 0.0;
    pid_speed[2].err_last = 0.0;
    pid_speed[2].integral = 0.0;
    pid_speed[2].Kp = 20;
    pid_speed[2].Ki = 0.0;
    pid_speed[2].Kd = 0.0;

    pid_speed[3].target_val = -3000.0;
    pid_speed[3].output_val = 0.0;
    pid_speed[3].err = 0.0;
    pid_speed[3].err_last = 0.0;
    pid_speed[3].integral = 0.0;
    pid_speed[3].Kp = 20;
    pid_speed[3].Ki = 0.0;
    pid_speed[3].Kd = 0.0;
}

/**
  * @name   location_pid_realize
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
  * @retval 通过PID计算后的输出
  */
static float location_pid_realize(PID *pid, float actual_val) {
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 设定闭环死区 */
    if ((pid->err >= -M2006_LOC_DEAD_ZONE) && (pid->err <= M2006_LOC_DEAD_ZONE)) {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*积分项，积分分离，偏差较大时去掉积分作用*/
    if (pid->err > -M2006_LOC_INTEGRAL_START_ERR && pid->err < M2006_LOC_INTEGRAL_START_ERR) {
        pid->integral += pid->err;
        /*积分范围限定，防止积分饱和*/
        if (pid->integral > M2006_LOC_INTEGRAL_MAX_VAL) {
            pid->integral = M2006_LOC_INTEGRAL_MAX_VAL;
        } else if (pid->integral < -M2006_LOC_INTEGRAL_MAX_VAL) {
            pid->integral = -M2006_LOC_INTEGRAL_MAX_VAL;
        }
    }

    /*PID算法实现*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_last);

    /*误差传递*/
    pid->err_last = pid->err;

    /*返回当前实际值*/
    return pid->output_val;
}

/**
  * @name   speed_pid_realize
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  *	@note 	无
  * @retval 通过PID计算后的输出
  */
static float speed_pid_realize(PID *pid, float actual_val) {
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 设定闭环死区 */
    if ((pid->err > -M2006_SPE_DEAD_ZONE) && (pid->err < M2006_SPE_DEAD_ZONE)) {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*积分项，积分分离，偏差较大时去掉积分作用*/
    if (pid->err > -M2006_SPE_INTEGRAL_START_ERR && pid->err < M2006_SPE_INTEGRAL_START_ERR) {
        pid->integral += pid->err;
        /*积分范围限定，防止积分饱和*/
        if (pid->integral > M2006_SPE_INTEGRAL_MAX_VAL) {
            pid->integral = M2006_SPE_INTEGRAL_MAX_VAL;
        } else if (pid->integral < -M2006_SPE_INTEGRAL_MAX_VAL) {
            pid->integral = -M2006_SPE_INTEGRAL_MAX_VAL;
        }
    }

    /*PID算法实现*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_last);

    /*误差传递*/
    pid->err_last = pid->err;

    /*返回当前实际值*/
    return pid->output_val;
}
