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

#include "ff.h"

FATFS fs;
FRESULT fr;
FIL fd;

extern USBD_HandleTypeDef hUsbDeviceHS;
extern int frame_count;
extern DMA_BUFFER uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];
extern int line_cnt;

DMA_BUFFER uint8_t mpu6000_data[14];
char is_cam_start = 0,is_imu_start = 0;

static void camera_img_send(void);

#define REQUEST_CAMERA_START    0xA0
#define REQUEST_CAMERA_STOP     0xA1

#define REQUEST_IMU_START       0xB0
#define REQUEST_IMU_STOP        0xB1

uint8_t camera_ctrl(USBD_SetupReqTypedef *req)
{
	uint8_t result = 'F';
	switch (req->bRequest)
	{
	case REQUEST_CAMERA_START:
		is_cam_start = 1;
		result = 'S';
		break;
	case REQUEST_CAMERA_STOP:
		is_cam_start = 0;
		result = 'S';
		break;
	case REQUEST_IMU_START:
		is_imu_start = 1;
		result = 'S';
		break;
	case REQUEST_IMU_STOP:
		is_imu_start = 0;
		result = 'S';
		break;	
	default:
		break;
	}

	return result;
}

void StartCameraTask(void const *argument)
{

	//
	//mpu6000_init();

	//	char filename[] = "test.txt";
	//	uint8_t write_dat[] = "	char hello[] = "hello";
	//
	//	while(1)
	//	{
	//		while(CDC_Transmit_HS(hello,sizeof(hello)) == USBD_BUSY);
	//
	//		osDelay(1000);
	//	}Hello,FATFS!\n";
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



	mt9v034_init();
	mpu6000_init();
	
	//int count = 5;
	//	while (count--)
	//	{
	//		osDelay(1000);
	//		printf("count %d\r\n", count);
	//	}
	
	
	
	while (1)
	{
		while (!is_cam_start)
		{
			osDelay(10);
		}

		dcmi_dma_start();
		camera_img_send();
		frame_count++;
	}
}

void camera_img_send(void)
{
#define SNED_SIZE (63 * 1024)

	for (uint32_t i = 0; i < FULL_IMAGE_SIZE; i += SNED_SIZE)
	{
		if (i + SNED_SIZE > FULL_IMAGE_SIZE)
		{
			while (CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i], FULL_IMAGE_SIZE - i) == USBD_BUSY)
				;
		}
		else
		{
			while (CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i], SNED_SIZE) == USBD_BUSY)
				;
		}
	}

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	while (hcdc->TxState != 0)
		;
}
