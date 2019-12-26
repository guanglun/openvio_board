#include "camera_task.h"
#include "openvio_def.h"
#include "openvio.h"

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



struct OPENVIO_STATUS vio_status;

static void camera_img_send(void);
static void camera_start_send(void);

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
		if(vio_status.cam_status == SENSOR_STATUS_WAIT)
		{
			result = 'S';
			vio_status.cam_status = SENSOR_STATUS_START;
		}else{
			result = 'F';
		}

		break;
	case REQUEST_CAMERA_STOP:
		if(vio_status.cam_status != SENSOR_STATUS_WAIT)
		{		
			result = 'S';
			vio_status.cam_status = SENSOR_STATUS_WAIT;
		}else{
			result = 'F';
		}
		
		break;
	case REQUEST_IMU_START:
		if(vio_status.imu_status == SENSOR_STATUS_WAIT)
		{			
			vio_status.imu_status = SENSOR_STATUS_START;
			result = 'S';
		}else{
			result = 'F';
		}
		break;
	case REQUEST_IMU_STOP:
		if(vio_status.imu_status != SENSOR_STATUS_WAIT)
		{		
			vio_status.imu_status = SENSOR_STATUS_WAIT;
			result = 'S';
		}else{
			result = 'F';
		}
		break;	
	default:
		break;
	}
	return result;
}

void StartCameraTask(void const *argument)
{
	int ret;
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


	openvio_status_init(&vio_status);
	
	mt9v034_init();
	mpu6000_init();
	
	//int count = 5;
	//	while (count--)
	//	{
	//		osDelay(1000);
	//		printf("count %d\r\n", count);
	//	}
	
//	while(1)
//	{
//		mpu6000_transmit();
//		osDelay(1000);
//	}

	//printf("dcmi_dma_start\r\n");
	
	while (1)
	{

		if(vio_status.cam_status == SENSOR_STATUS_START)
		{
			camera_start_send();
			
		}else if(vio_status.cam_status == SENSOR_STATUS_RUNNING)
		{
			dcmi_dma_start();
			camera_img_send();
		}

		mpu6000_transmit();
		frame_count++;
		
		//osDelay(4000);
	}
}

#define SNED_SIZE (63 * 1024)

void camera_start_send(void)
{
//	int ret;
//	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
//			uint8_t start_byte[4] = {0x12,0x34,0x56,0x78};
//			while (hcdc->TxState != 0);

//			do{
//				ret = CDC_Transmit_HS(start_byte, 4);
//			}while(ret == USBD_BUSY);
//			if(ret == USBD_OK)
			{
				vio_status.cam_status = SENSOR_STATUS_RUNNING;
			}	
}

void camera_img_send(void)
{
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	for (uint32_t i = 0; i < FULL_IMAGE_SIZE; i += SNED_SIZE)
	{
		if (i + SNED_SIZE > FULL_IMAGE_SIZE)
		{
			openvio_usb_send(SENSOR_USB_CAM,&dcmi_image_buffer_8bit_1[i], FULL_IMAGE_SIZE - i);
		}
		else
		{
			openvio_usb_send(SENSOR_USB_CAM,&dcmi_image_buffer_8bit_1[i], SNED_SIZE);
		}
		
		mpu6000_transmit();
	}
}
