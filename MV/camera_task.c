#include "camera_task.h"
#include "cambus.h"
#include "mt9v034.h"
#include "dcmi.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "mpu6000.h"

//#include "ff.h"

//FATFS   fs;         
//FRESULT fr;       
//FIL     fd;   

extern USBD_HandleTypeDef hUsbDeviceHS;
extern int frame_count;
extern DMA_BUFFER uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];
extern int line_cnt;

  

void StartCameraTask(void const * argument)
{
	
	mpu6000_init();
//	char filename[] = "test.txt";
//	uint8_t write_dat[] = "Hello,FATFS!\n";
//	uint16_t write_num = 0;
//	
//	fr = f_mount(&fs, "", 0);
//	if(fr == FR_OK)
//	{
//		printf("SD card mount ok!\r\n");
//	}
//	else
//	{
//		printf("SD card mount error, error code:%d.\r\n",fr);
//	}
//	
//	fr = f_open(&fd, filename, FA_CREATE_ALWAYS | FA_WRITE);
//	if(fr == FR_OK)
//	{
//		printf("open file \"%s\" ok! \r\n", filename);
//	}
//	else
//	{
//	printf("open file \"%s\" error : %d\r\n", filename, fr);
//	}

//	fr = f_write(&fd, write_dat, sizeof(write_dat), (void *)&write_num);
//	if(fr == FR_OK)
//	{
//		printf("write %d dat to file \"%s\" ok,dat is \"%s\".\r\n", write_num, filename, write_dat);
//	}
//	else
//	{
//		printf("write dat to file \"%s\" error,error code is:%d\r\n", filename, fr);
//	}

//	fr = f_close(&fd);
//	if(fr == FR_OK)
//	{
//		printf("close file \"%s\" ok!\r\n", filename);
//	}
//	else
//	{
//		printf("close file \"%s\" error, error code is:%d.\r\n", filename, fr);
//	}
	
	

	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	
	int ret = cambus_scan();

	printf("cambus_scan %02X\r\n", ret);

	mt9v034_init();
	osDelay(10);
	
	int count = 5;

	while(count--)
	{
		osDelay(1000);
		printf("count %d\r\n",count);
	}

	dcmi_dma_start();

	while(1)
	{
		//printf("%d\r\n",frame_count);
		frame_count++;
		//osDelay(1000);
		//if(frame_count > 0)
		{
			//printf("send start\r\n");
			frame_count = 0;
			#define SNED_SIZE (63*1024)
//			if(line_cnt == 240)
//			{
				for(uint32_t i = 0;i<FULL_IMAGE_SIZE;i += SNED_SIZE)
				{
					if( i + SNED_SIZE > FULL_IMAGE_SIZE)
					{
						while(CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i],FULL_IMAGE_SIZE - i) == USBD_BUSY);
					}else{
						while(CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i],SNED_SIZE) == USBD_BUSY);
					}
				}
//			}

			USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
			while(hcdc->TxState != 0);
			//printf("send end\r\n");
			
			osDelay(20);
			dcmi_dma_start();
		}
		
	}
}
