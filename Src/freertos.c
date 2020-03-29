/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "camera_task.h"
#include "ff.h"
#include "dcmi.h"

#include "openvio.h"

FRESULT fr;
DMA_BUFFER FATFS fs;
DMA_BUFFER FIL fd;

extern USBD_HandleTypeDef hUsbDeviceHS;
extern struct OPENVIO_STATUS vio_status;
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId cameraTaskHandle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(cameraTask, StartCameraTask, osPriorityNormal, 0, 1024 * 2);
  cameraTaskHandle = osThreadCreate(osThread(cameraTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for FATFS */
  MX_FATFS_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  char filename[] = "test.txt";
  uint8_t write_dat[] = "hello";

  uint16_t write_num = 0;

  fr = f_mount(&fs, "0:/", 1);
  if (fr == FR_OK)
  {
    printf("SD card mount ok!\r\n");
  }
  else
  {
    printf("SD card mount error, error code:%d.\r\n", fr);
  }

  fr = f_open(&fd, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr == FR_OK)
  {
    printf("open file \"%s\" ok! \r\n", filename);
  }
  else
  {
    printf("open file \"%s\" error : %d\r\n", filename, fr);
  }

  fr = f_write(&fd, write_dat, sizeof(write_dat), (void *)&write_num);
  if (fr == FR_OK)
  {
    printf("write %d dat to file \"%s\" ok,dat is \"%s\".\r\n", write_num, filename, write_dat);
  }
  else
  {
    printf("write dat to file \"%s\" error,error code is:%d\r\n", filename, fr);
  }

  fr = f_close(&fd);
  if (fr == FR_OK)
  {
    printf("close file \"%s\" ok!\r\n", filename);
  }
  else
  {
    printf("close file \"%s\" error, error code is:%d.\r\n", filename, fr);
  }
  
  uint32_t connect_delay = 0;
  
  for (;;)
  {

	  	if(hUsbDeviceHS.dev_state != HAL_PCD_STATE_BUSY && vio_status.usb_status == USB_CONNECT)
		{
			connect_delay++;
			if(connect_delay >= 100)
			{
				connect_delay = 0;
				printf("[USB DISCONNECT]\r\n");
				vio_status.usb_status = USB_DISCONNECT;
        MX_USB_DEVICE_Init();
			}

		}else if(hUsbDeviceHS.dev_state == HAL_PCD_STATE_BUSY && vio_status.usb_status == USB_DISCONNECT)
		{
			connect_delay++;
			if(connect_delay >= 100)
			{			
				connect_delay = 0;
				printf("[USB CONNECT]\r\n");
				vio_status.usb_status = USB_CONNECT;
			}
		}
		
	osDelay(10);
    // printf("status:%d\r\n",hUsbDeviceHS.dev_state);
    // osDelay(10);

//    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
//    osDelay(1000);    
//    HAL_GPIO_WritePin(LED_E9_GPIO_Port, LED_E9_Pin, GPIO_PIN_RESET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_E9_GPIO_Port, LED_E9_Pin, GPIO_PIN_SET);
//    osDelay(1000);     
//    HAL_GPIO_WritePin(LED_E10_GPIO_Port, LED_E10_Pin, GPIO_PIN_RESET);
//    osDelay(1000);
//    HAL_GPIO_WritePin(LED_E10_GPIO_Port, LED_E10_Pin, GPIO_PIN_SET);
//    osDelay(1000);            
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
