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
TaskHandle_t selectableTaskHandleList[10];
uint8_t currentSelectedTaskIndex = 0;
uint8_t selectableTaskHandleNumber = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LedTask(void *pvParameters);

void IMUTask(void *pvParameters);

void SelectionTask(void *pvParameters);

void DebugTask(void *pvParameters);

void ServoTask(void *pvParameters);

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

    TaskHandle_t IMUTaskHandle = NULL;
    xTaskCreate(IMUTask, "IMUTask", 128, NULL, 6, &IMUTaskHandle);

    TaskHandle_t SelectionTaskHandle = NULL;
    xTaskCreate(SelectionTask, "SelectionTask", 128, NULL, 5, &SelectionTaskHandle);

    TaskHandle_t DebugTaskHandle = NULL;
    xTaskCreate(DebugTask, "Debug", 128, NULL, 5, &DebugTaskHandle);
    vTaskSuspend(DebugTaskHandle);

    TaskHandle_t ServoTaskHandle = NULL;
    xTaskCreate(ServoTask, "Servo", 128, NULL, 5, &ServoTaskHandle);
    vTaskSuspend(ServoTaskHandle);

    selectableTaskHandleList[0] = DebugTaskHandle;
    selectableTaskHandleList[1] = ServoTaskHandle;
    selectableTaskHandleNumber = 2;
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
 * @brief LED blink task, showing MCU is working correctly
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
 * @brief read data from IMU
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
 * @brief Select any other tasks
 * @param pvParameters
 */
void SelectionTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (button_confirm_press_pending_flag) { // confirm
            button_reset_all_flags();

            gui_task_selection(pcTaskGetName(selectableTaskHandleList[currentSelectedTaskIndex]), 1);

            vTaskResume(selectableTaskHandleList[currentSelectedTaskIndex]);
            vTaskSuspend(NULL);
        } else if (button_left_press_pending_flag) { // left: reduce
            button_reset_all_flags();
            if (currentSelectedTaskIndex == 0) {
                currentSelectedTaskIndex = selectableTaskHandleNumber - 1;
            } else {
                currentSelectedTaskIndex--;
            }
        } else if (button_right_press_pending_flag) { // right: increase
            button_reset_all_flags();
            currentSelectedTaskIndex++;
            currentSelectedTaskIndex %= selectableTaskHandleNumber;
        } else {
            gui_task_selection(pcTaskGetName(selectableTaskHandleList[currentSelectedTaskIndex]), 0);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Show debug information in screen
 * @param pvParameters
 */
void DebugTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (button_cancel_press_pending_flag) { // Back to selection task
            button_reset_all_flags();

            vTaskResume(xTaskGetHandle("SelectionTask"));
            vTaskSuspend(NULL);
        }

        gui_show_variables("X", (int16_t) mpu6050.KalmanAngleX,
                           "Y", (int16_t) mpu6050.KalmanAngleY,
                           "Gyro_Z", (int16_t) mpu6050.Gz);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief only control servo
 * @param pvParameters
 */
void ServoTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    int16_t servo1 = 0, servo2 = 0;
    while (1) {
        if (button_cancel_press_pending_flag) { // Back to selection task
            button_reset_all_flags();

            vTaskResume(xTaskGetHandle("SelectionTask"));
            vTaskSuspend(NULL);
        } else if (button_left_press_pending_flag) {
            button_reset_all_flags();
            servo1--;
            servo2--;
        } else if (button_right_press_pending_flag) {
            button_reset_all_flags();
            servo1++;
            servo2++;
        }
        gui_control_servo(servo1, SERVO1_MAX_DEGREE, servo2, SERVO2_MAX_DEGREE);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
