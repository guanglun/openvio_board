#ifndef __OPENVIO_DEF_H__
#define __OPENVIO_DEF_H__

#include "main.h"

#define DMA_BUFFER \
  __attribute__((section(".RAM_D1")))
  
enum SENSOR_USB{
	SENSOR_USB_CAM,
	SENSOR_USB_IMU
};

enum SENSOR_STATUS{
    SENSOR_STATUS_WAIT,
    SENSOR_STATUS_START,
    SENSOR_STATUS_RUNNING
};

enum USB_LOCK_STATUS{
    USB_LOCK_NULL,
    USB_LOCK_CAM,
    USB_LOCK_IMU
};

enum USB_CONNECT_STATUS{
    USB_DISCONNECT,
    USB_CONNECT
};

enum CAMERA_NAME{
    MT9V034,
    OV7725
};

struct OPENVIO_STATUS{
	enum SENSOR_STATUS cam_status;
	enum SENSOR_STATUS imu_status;
    enum USB_CONNECT_STATUS usb_status;
    uint8_t usb_lock_status;
	uint8_t is_imu_send;

    uint8_t cam_id;
    uint8_t gs_bpp;
    uint8_t cam_frame_size_num;
    uint32_t cam_frame_size;
    uint8_t cam_name;
};



#endif

