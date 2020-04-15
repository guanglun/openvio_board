#ifndef __OPENVIO_DEF_H__
#define __OPENVIO_DEF_H__

#include "main.h"

#define DMA_BUFFER \
  __attribute__((section(".RAM_D1")))

#define CAM_PACKAGE_MAX_SIZE (63*1024)
#define USB_DMA_PACKAGE_SIZE (63*1024)

enum SENSOR_USB{
	SENSOR_USB_CAM = 0,
	SENSOR_USB_IMU
};

enum SENSOR_STATUS{
    SENSOR_STATUS_WAIT = 0,
    SENSOR_STATUS_START,
    SENSOR_STATUS_RUNNING
};

enum USB_LOCK_STATUS{
    USB_LOCK_NULL = 0,
    USB_LOCK_CAM,
    USB_LOCK_IMU
};

enum USB_CONNECT_STATUS{
    USB_DISCONNECT = 0,
    USB_CONNECT
};

enum CAMERA_NAME{
    MT9V034 = 0,
    OV7725
};

enum USB_STATUS{
    USB_WAIT = 0,
    USB_WORKING
};

struct USB_STRUCT{
    uint32_t target_len;
    uint32_t len;
    uint8_t *addr;
    enum USB_STATUS status;
};

struct OPENVIO_STATUS{
    struct USB_STRUCT usb_s;
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

