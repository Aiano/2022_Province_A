/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "i2c.h"
#include "gui.h"
#include "mpu6050.h"
#include "servo.h"
#include "bsp_button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
MPU6050_t mpu6050;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LedTask(void *pvParameters);

void ScreenTask(void *pvParameters);

void IMUTask(void *pvParameters);

void ManageTask(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    TaskHandle_t LedTaskHandle = NULL;
    xTaskCreate(LedTask, "LedTask", 128, NULL, 1, &LedTaskHandle);

    TaskHandle_t ScreenTaskHandle = NULL;
    xTaskCreate(ScreenTask, "ScreenTask", 128, NULL, 2, &ScreenTaskHandle);

    TaskHandle_t IMUTaskHandle = NULL;
    xTaskCreate(IMUTask, "IMUTask", 128, NULL, 3, &IMUTaskHandle);

    TaskHandle_t ManageTaskHandle = NULL;
    xTaskCreate(ManageTask, "ManageTask", 128, NULL, 6, &ManageTaskHandle);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief LED闪烁任务，正常闪烁代表单片机正常运行
 * @param pvParameters
 */
void LedTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/**
 * @brief 刷新屏幕任务
 * @param pvParameters
 */
void ScreenTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
//        gui_show_variables("X", (int16_t) mpu6050.KalmanAngleX,
//                           "Y", (int16_t) mpu6050.KalmanAngleY,
//                           "Gyro_Z", (int16_t) mpu6050.Gz);

        for (uint16_t i = 0; i <= 180; i++) {
            servo_set_degree(120,0);
            gui_control_servo(120, SERVO1_MAX_DEGREE, i, SERVO2_MAX_DEGREE);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
        }


    }

}

/**
 * @brief 读取来自IMU数据的任务
 * @param pvParameters
 */
void IMUTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        taskENTER_CRITICAL();
        MPU6050_Read_All(&hi2c1, &mpu6050);
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief 负责管理和切换其它FreeRTOS任务的任务
 * @param pvParameters
 */
void ManageTask(void *pvParameters){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while(1){
        if(button_left_press_pending_flag){
            button_left_press_pending_flag = 0; // reset flag

            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
