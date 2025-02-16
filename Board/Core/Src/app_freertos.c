/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "interface_uart.h"
#include "board.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint32_t all_error;
extern float vbus_voltage;
extern Iph_ABC_t current_meas_;

extern float phase_;        // [rad]
extern float phase_vel_;    // [rad/s]
extern float pos_estimate_; // [turn]
extern float vel_estimate_; // [turn/s]
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RGB_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId info_taskHandle;
osThreadId control_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartTask_RGB(void const *argument);
void StartTask_LED(void const *argument);
void start_info_task(void const *argument);
void start_control_task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
    task. It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()). If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

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
    osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 256);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of RGB_Task */
    osThreadDef(RGB_Task, StartTask_RGB, osPriorityBelowNormal, 0, 256);
    RGB_TaskHandle = osThreadCreate(osThread(RGB_Task), NULL);

    /* definition and creation of LED_Task */
    osThreadDef(LED_Task, StartTask_LED, osPriorityLow, 0, 256);
    LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

    /* definition and creation of info_task */
    osThreadDef(info_task, start_info_task, osPriorityNormal, 0, 1024);
    info_taskHandle = osThreadCreate(osThread(info_task), NULL);

    /* definition and creation of control_task */
    osThreadDef(control_task, start_control_task, osPriorityAboveNormal, 0, 256);
    control_taskHandle = osThreadCreate(osThread(control_task), NULL);

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
void StartDefaultTask(void const *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) { osDelay(1); }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_RGB */
/**
 * @brief Function implementing the RGB_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_RGB */
void StartTask_RGB(void const *argument)
{
    /* USER CODE BEGIN StartTask_RGB */
    /* Infinite loop */
    for (;;) { osDelay(1); }
    /* USER CODE END StartTask_RGB */
}

/* USER CODE BEGIN Header_StartTask_LED */
/**
 * @brief Function implementing the LED_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_LED */
void StartTask_LED(void const *argument)
{
    /* USER CODE BEGIN StartTask_LED */
    /* Infinite loop */
    for (;;) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(500);
    }
    /* USER CODE END StartTask_LED */
}

/* USER CODE BEGIN Header_start_info_task */
/**
 * @brief Function implementing the info_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_info_task */
void start_info_task(void const *argument)
{
    /* USER CODE BEGIN start_info_task */
    /* Infinite loop */
    for (;;) {
        usart_printf("CCR:%d, %.3f, %d, %d, %d, %f, %f, %f, %f, %f, %f, %f \n", all_error, vbus_voltage, TIM1->CCR1,
                     TIM1->CCR2, TIM1->CCR3, current_meas_.phA, current_meas_.phB, current_meas_.phC, phase_,
                     phase_vel_, pos_estimate_, vel_estimate_);
        osDelay(1);
    }
    /* USER CODE END start_info_task */
}

/* USER CODE BEGIN Header_start_control_task */
/**
 * @brief Function implementing the control_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_control_task */
void start_control_task(void const *argument)
{
    /* USER CODE BEGIN start_control_task */
    /* Infinite loop */
    for (;;) { osDelay(1); }
    /* USER CODE END start_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
