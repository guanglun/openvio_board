#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"

#define OV2640_SLV_ADDR     (0x60)
#define OV5640_SLV_ADDR     (0x78)
#define OV7725_SLV_ADDR     (0x42)
#define MT9V034_SLV_ADDR    (0xB8)
#define LEPTON_SLV_ADDR     (0x54)
#define HM01B0_SLV_ADDR     (0x48)

// Chip ID Registers
#define OV5640_CHIP_ID      (0x300A)
#define OV_CHIP_ID          (0x0A)
#define ON_CHIP_ID          (0x00)
#define HIMAX_CHIP_ID       (0x0001)

// Chip ID Values
#define OV2640_ID           (0x26)
#define OV5640_ID           (0x56)
#define OV7690_ID           (0x76)
#define OV7725_ID           (0x77)
#define OV9650_ID           (0x96)
#define MT9V034_ID          (0x13)
#define LEPTON_ID           (0x54)
#define HM01B0_ID           (0xB0)

void camera_init(void);
void dcmi_dma_start(void);

#endif
