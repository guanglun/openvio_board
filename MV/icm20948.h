#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "main.h"


int icm20948_init(void);
void icm20948_read(uint8_t *buf);
void icm20948_transmit(void);

#endif

