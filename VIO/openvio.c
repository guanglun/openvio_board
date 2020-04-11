#include "openvio.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include "mt9v034.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

// typedef enum {
//     FRAMESIZE_INVALID = 0,
//     // C/SIF Resolutions
//     FRAMESIZE_QQCIF,    // 88x72
//     FRAMESIZE_QCIF,     // 176x144
//     FRAMESIZE_CIF,      // 352x288
//     FRAMESIZE_QQSIF,    // 88x60
//     FRAMESIZE_QSIF,     // 176x120
//     FRAMESIZE_SIF,      // 352x240
//     // VGA Resolutions
//     FRAMESIZE_QQQQVGA,  // 40x30
//     FRAMESIZE_QQQVGA,   // 80x60
//     FRAMESIZE_QQVGA,    // 160x120
//     FRAMESIZE_QVGA,     // 320x240
//     FRAMESIZE_VGA,      // 640x480
//     FRAMESIZE_HQQQVGA,  // 60x40
//     FRAMESIZE_HQQVGA,   // 120x80
//     FRAMESIZE_HQVGA,    // 240x160
//     // FFT Resolutions
//     FRAMESIZE_64X32,    // 64x32
//     FRAMESIZE_64X64,    // 64x64
//     FRAMESIZE_128X64,   // 128x64
//     FRAMESIZE_128X128,  // 128x128
//     // Other
//     FRAMESIZE_LCD,      // 128x160
//     FRAMESIZE_QQVGA2,   // 128x160
//     FRAMESIZE_WVGA,     // 720x480
//     FRAMESIZE_WVGA2,    // 752x480
//     FRAMESIZE_SVGA,     // 800x600
//     FRAMESIZE_XGA,      // 1024x768
//     FRAMESIZE_SXGA,     // 1280x1024
//     FRAMESIZE_UXGA,     // 1600x1200
// } framesize_t;

void openvio_status_init(struct OPENVIO_STATUS *status)
{
    status->cam_status = 0;
    status->imu_status = 0;
    status->usb_lock_status = 0;
	status->is_imu_send = 0;
	status->usb_status = USB_DISCONNECT;

	status->cam_name = MT9V034;
	status->cam_frame_size_num = FRAMESIZE_QVGA;

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
			vio_status.cam_status = SENSOR_STATUS_WAIT;
			printf("cam error\r\n");
		}else if(usb == SENSOR_USB_IMU)
		{
			vio_status.imu_status = SENSOR_STATUS_WAIT;
			printf("imu error\r\n");
		}
	}
	
	if(try_cnt >= USB_TRY_NUM)
	{
		
		hcdc->TxState = USBD_OK;
		
		if(usb == SENSOR_USB_CAM)
		{
			vio_status.cam_status = SENSOR_STATUS_WAIT;
			printf("cam timeout\r\n");
		}else if(usb == SENSOR_USB_IMU)
		{
			vio_status.imu_status = SENSOR_STATUS_WAIT;
			printf("imu  timeout\r\n");
		}
	
	}
}

