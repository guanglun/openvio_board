#include "openvio.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

void openvio_status_init(struct OPENVIO_STATUS *status)
{
    status->cam_status = 0;
    status->imu_status = 0;
    status->usb_lock_status = 0;
	status->is_imu_send = 0;
}

#define USB_TRY_NUM 65500
void openvio_usb_send(enum SENSOR_USB usb,uint8_t* Buf, uint16_t Len)
{
	int ret = USBD_FAIL,try_cnt = 0;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
	
	do{
		if(usb == SENSOR_USB_CAM)
		{
			ret = CDC_Transmit_HS(Buf,Len);
		}else if(usb == SENSOR_USB_IMU)
		{
			ret = MPU_Transmit_HS(Buf,Len);
		}
		try_cnt++;
		osDelay(1);
	}
	while(ret == USBD_BUSY && try_cnt < USB_TRY_NUM);
	
	if(ret == USBD_FAIL)
	{
		if(usb == SENSOR_USB_CAM)
		{
			printf("cam error\r\n");
		}else if(usb == SENSOR_USB_IMU)
		{
			printf("imu error\r\n");
		}
	}
	
	if(try_cnt >= USB_TRY_NUM)
	{
		
		hcdc->TxState = USBD_OK;
		
		if(usb == SENSOR_USB_CAM)
		{
			printf("cam timeout\r\n");
		}else if(usb == SENSOR_USB_IMU)
		{
			printf("imu  timeout\r\n");
		}
	
	}
}

