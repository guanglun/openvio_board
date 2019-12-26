#ifndef __OPENVIO_H__
#define __OPENVIO_H__

#include "openvio_def.h"




void openvio_status_init(struct OPENVIO_STATUS *status);
void openvio_usb_send(enum SENSOR_USB usb,uint8_t* Buf, uint16_t Len);

#endif
