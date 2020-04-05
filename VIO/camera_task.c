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

#include "icm20948.h"
#include "camera.h"


extern USBD_HandleTypeDef hUsbDeviceHS;
extern int frame_count;
extern DMA_BUFFER uint8_t dcmi_image_buffer[FULL_IMAGE_SIZE];
extern int line_cnt;

DMA_BUFFER uint8_t mpu6000_data[14];



struct OPENVIO_STATUS vio_status;

static void camera_img_send(void);
static void camera_start_send(void);

#define REQUEST_CAMERA_START    				0xA0
#define REQUEST_CAMERA_STOP     				0xA1
#define REQUEST_CAMERA_SET_FRAME_SIZE_NUM     	0xA2

#define REQUEST_IMU_START       0xB0
#define REQUEST_IMU_STOP        0xB1

uint8_t camera_ctrl(USBD_SetupReqTypedef *req,uint8_t *s_data)
{
	uint8_t s_len = 1;
	s_data[0] = 'F';
	switch (req->bRequest)
	{
	case REQUEST_CAMERA_START:
		if(vio_status.cam_status == SENSOR_STATUS_WAIT)
		{
			s_data[0] = 'S';
			s_data[1] = vio_status.cam_frame_size_num;
			s_len = 2;
			vio_status.cam_status = SENSOR_STATUS_START;
		}else{
			s_data[0] = 'F';
		}

		break;
	case REQUEST_CAMERA_STOP:
		if(vio_status.cam_status != SENSOR_STATUS_WAIT)
		{		
			s_data[0] = 'S';
			vio_status.cam_status = SENSOR_STATUS_WAIT;
		}else{
			s_data[0] = 'F';
		}
		
		break;
	case REQUEST_IMU_START:
		if(vio_status.imu_status == SENSOR_STATUS_WAIT)
		{			
			vio_status.imu_status = SENSOR_STATUS_START;
			s_data[0] = 'S';
		}else{
			s_data[0] = 'F';
		}
		break;
	case REQUEST_IMU_STOP:
		if(vio_status.imu_status != SENSOR_STATUS_WAIT)
		{		
			vio_status.imu_status = SENSOR_STATUS_WAIT;
			s_data[0] = 'S';
		}else{
			s_data[0] = 'F';
		}
		break;	
	case REQUEST_CAMERA_SET_FRAME_SIZE_NUM:
			mt9v034_config(req->wValue);
			s_data[0] = 'S';
			s_data[1] = vio_status.cam_frame_size_num;
			s_len = 2;
		break;	
		
	default:
		break;
	}
	return s_len;
}

void StartCameraTask(void const *argument)
{
	int ret;

	openvio_status_init(&vio_status);
	
	mt9v034_init();
	icm20948_init();
	
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
	uint8_t isCamReady = 0;
	TickType_t xTimeNow = 0,xTimeLast = 0;
	
	
	while (1)
	{
		if(vio_status.cam_status == SENSOR_STATUS_START)
		{
			//osDelay(100);
			openvio_usb_send(SENSOR_USB_CAM,"START", 5);
			camera_start_send();

			//osDelay(1000);
			isCamReady = 0;
			xTimeLast = xTimeNow;
			
		}else if(vio_status.cam_status == SENSOR_STATUS_RUNNING)
		{
			if(isCamReady == 0)
			{
				dcmi_dma_start();
				isCamReady = 1;
			}
			
			xTimeNow = xTaskGetTickCount();

			//50 20Hz
			if((xTimeNow-xTimeLast) >= 0 && isCamReady == 1)
			{
				xTimeLast = xTimeNow;
				//printf("time:%d\r\n",xTimeNow);
				
				camera_img_send();
				isCamReady = 0;
			}			
			
		}

		icm20948_transmit();

		frame_count++;
	}
}

#define USB_SEND_MAX_SIZE (63 * 1024)

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
	openvio_usb_send(SENSOR_USB_CAM,"CAM", 3);
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	for (uint32_t i = 0; i < vio_status.cam_frame_size; i += USB_SEND_MAX_SIZE)
	{
		if (i + USB_SEND_MAX_SIZE > vio_status.cam_frame_size)
		{
			openvio_usb_send(SENSOR_USB_CAM,&dcmi_image_buffer[i], vio_status.cam_frame_size - i);
		}
		else
		{
			openvio_usb_send(SENSOR_USB_CAM,&dcmi_image_buffer[i], USB_SEND_MAX_SIZE);
		}
		
		icm20948_transmit();
		
	}
}
