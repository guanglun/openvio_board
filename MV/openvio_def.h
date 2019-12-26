#ifndef __OPENVIO_DEF_H__
#define __OPENVIO_DEF_H__

#include "main.h"

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

struct OPENVIO_STATUS{
	enum SENSOR_STATUS cam_status;
	enum SENSOR_STATUS imu_status;
    uint8_t usb_lock_status;
	uint8_t is_imu_send;
};



#endif

