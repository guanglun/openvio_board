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
#include "openvio_task.h"
#include "dcmi.h"
#include "sd_card.h"
#include "openvio.h"


uint16_t adc_value;
uint8_t fps_value;
extern uint8_t fps_count;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern struct OPENVIO_STATUS vio_status;
extern ADC_HandleTypeDef hadc1;
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
osThreadId openvioTaskHandle;
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
  osThreadDef(openvioTask, StartOpenvioTask, osPriorityNormal, 0, 1024 * 2);
  openvioTaskHandle = osThreadCreate(osThread(openvioTask), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
DMA_BUFFER uint16_t ADC_ConvertedValue[10];
uint16_t Get_Adc(uint32_t ch)   
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    ADC1_ChanConf.Channel=ch;                                   //ͨ��
    ADC1_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1������
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//����ʱ��       
	ADC1_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//���߲ɼ�          		
	ADC1_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
	ADC1_ChanConf.Offset=0;   
    HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf);        //ͨ������

    HAL_ADC_Start(&hadc1);                               //����ADC
	
    HAL_ADC_PollForConversion(&hadc1,10);                //��ѯת��
	return (uint16_t)HAL_ADC_GetValue(&hadc1);	            //�������һ��ADC1�������ת�����
}

uint16_t Get_Adc_Average(uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		HAL_Delay(5);
	}
	return temp_val/times;
} 

QueueHandle_t xQueue;

extern DMA_BUFFER uint8_t dcmi_image_buffer[CAM_PACKAGE_MAX_SIZE * 2];
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
	

	
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
	xQueue = xQueueCreate( 30, sizeof(struct USB_FRAME_STRUCT) );
	struct USB_FRAME_STRUCT usb_frame_s;
  for (;;)
  {
    if( xQueueReceive( xQueue, &(usb_frame_s), ( TickType_t ) 0xFFFFFF ) )
        {
            while (CAM_Transmit_HS(usb_frame_s.addr,usb_frame_s.len) != 0)
			{
				//osDelay(1);
			}
        }

    //	  	if(hUsbDeviceHS.dev_state != HAL_PCD_STATE_BUSY && vio_status.usb_status == USB_CONNECT)
    //		{
    //			connect_delay++;
    //			if(connect_delay >= 100)
    //			{
    //				connect_delay = 0;
    //				printf("[USB DISCONNECT]\r\n");
    //				vio_status.usb_status = USB_DISCONNECT;
    //        MX_USB_DEVICE_Init();
    //			}

    //		}else if(hUsbDeviceHS.dev_state == HAL_PCD_STATE_BUSY && vio_status.usb_status == USB_DISCONNECT)
    //		{
    //			connect_delay++;
    //			if(connect_delay >= 100)
    //			{
    //				connect_delay = 0;
    //				printf("[USB CONNECT]\r\n");
    //				vio_status.usb_status = USB_CONNECT;
    //			}
    //		}

    
//	HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, 200);
//    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
//    {
//      uint32_t AD_Value = HAL_ADC_GetValue(&hadc1);
//      printf("%d\t%0.2f\r\n", AD_Value,3.3f*(float)AD_Value/4096*2);
//    }
//	HAL_ADC_Stop(&hadc1);
	
//	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1000);
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	
	
	
	
	
	
//	adc_value=Get_Adc_Average(ADC_CHANNEL_16,20);
//	//printf("%d\t%0.2f\t%0.2f\r\n", value,3.3f*(float)value/4096,3.3f*(float)value/4096*2);
//	HAL_Delay(1000);
//	fps_value = fps_count;
//	printf("%d\r\n",fps_count);
//	fps_count=0;
	
	
	
	
	
//	value=0;
//	for(int i=0;i<10;i++)
//	{
//		value+=ADC_ConvertedValue[i];
//	}
//	value/=10;
//	printf("%d\t%0.2f\r\n", value,3.3f*(float)value/4096*2);
	
//    GPIO_PinState state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

//    printf(" %d\r\n", state);

//    osDelay(100);


    
//    osDelay(1000);
//	
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
        //osDelay(1000);
		
		
		
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* ��ȡ���? */
    ADC_ConvertedValue[0] = HAL_ADC_GetValue(AdcHandle); 
	printf("%d\t%0.2f\r\n", ADC_ConvertedValue[0],3.3f*(float)ADC_ConvertedValue[0]/4096);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
