#include "openvio.h"

void openvio_status_init(struct OPENVIO_STATUS *status)
{
    status->is_cam_start = 0;
    status->is_imu_start = 0;
    status->usb_lock_status = 0;
	status->is_imu_send = 0;
}