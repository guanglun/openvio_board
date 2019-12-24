#ifndef __OPENVIO_DEF_H__
#define __OPENVIO_DEF_H__

#include "main.h"

enum USB_LOCK_STATUS{
    USB_LOCK_NULL,
    USB_LOCK_CAM,
    USB_LOCK_IMU
};

struct OPENVIO_STATUS{
	uint8_t is_cam_start;
	uint8_t is_imu_start;
    uint8_t usb_lock_status;
	uint8_t is_imu_send;
};

#endif

