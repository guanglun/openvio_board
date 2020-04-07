/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <string.h>
#include "mt9v034.h"
#include "main.h"
#include "cambus.h"
#include "openvio.h"

extern I2C_HandleTypeDef hi2c1;

struct mt9v034_reg
{
	uint8_t addr;
	uint16_t val;
};
#define MT9V034_TABLE_END 0xff
#define MT9V034_TABLE_WAIT_MS 0
enum
{
	MT9V034_MODE_640x480,
	MT9V034_MODE_752x480,
};

#define MT9V034_IDRegister 0x00
#define MT9V034_ColumnStart 0x01
#define MT9V034_RowStart 0x02
#define MT9V034_WindowHeight 0x03
#define MT9V034_WindowWidth 0x04
#define MT9V034_HorizontalBlanking 0x05
#define MT9V034_VerticalBlanking 0x06
#define MT9V034_CoarseShutterWidth1 0x08
#define MT9V034_CoarseShutterWidth2 0x09
#define MT9V034_CoarseShutterWidthControl 0x0A
#define MT9V034_CoarseShutterWidthTotal 0x0B
#define MT9V034_FineShutterWidth1 0xD3
#define MT9V034_FineShutterWidth2 0xD4
#define MT9V034_FineShutterWidthTotal 0xD5
#define MT9V034_ReadMode 0x0D
#define MT9V034_HighDynamicRangeEnable 0x0F
#define MT9V034_ADCResolutionControl 0x1C
#define MT9V034_V1Control 0x31
#define MT9V034_V2Control 0x32
#define MT9V034_V3Control 0x33
#define MT9V034_V4Control 0x34
#define MT9V034_AnalogGainControl 0x35
#define MT9V034_RowNoiseCorrectionControl1 0x70
#define MT9V034_TiledDigitalGain 0x80
#define MT9V034_AECorAGCEnable 0xAF
#define MT9V034_R0x20 0x20
#define MT9V034_R0x24 0x24
#define MT9V034_R0x2B 0x2B
#define MT9V034_R0x2F 0x2F

#define CONFIG_FINISH 12

#define msleep HAL_Delay

static uint16_t mt9v034_ReadReg16(uint8_t address);
static uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data);
static int set_auto_exposure(int enable, int exposure_us);
static int set_framesize(framesize_t framesize);
static int set_colorbar(int enable); //是否使能测试条码
static int set_hmirror(int enable); //设置水平对称
static int set_vflip(int enable); //设置竖直对称

int mt9v034_init(void)
{
	uint16_t chip_version = 0;
	int retry_num = 1, err = 0;

	////////////////////////////////////////////////////////////////////////////////
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	int id = cambus_scan();

	printf("cambus_scan %02X\r\n", id);
	////////////////////////////////////////////////////////////////////////////////

	while (!chip_version && retry_num-- > 0)
		chip_version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);

	if (chip_version != 0x1324)
	{
		printf("read err\r\n");
		return -1;
	}
	printf("[mt9v034]: ok\r\n");

	mt9v034_config(vio_status.cam_frame_size_num);

	set_colorbar(0);
	set_vflip(0);
	set_hmirror(0);
	//set_auto_exposure(0,0);

	uint16_t chip_control;
	int enable = 0;
	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL, &chip_control);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL,
						 (chip_control & (~MT9V034_CHIP_CONTROL_MODE_MASK)) | ((enable != 0) ? MT9V034_CHIP_CONTROL_SNAP_MODE : MT9V034_CHIP_CONTROL_MASTER_MODE));


	return err;
}

const int resolution[][2] = {
	{0, 0},
	// C/SIF Resolutions
	{88, 72},	/* QQCIF     */
	{176, 144}, /* QCIF      */
	{352, 288}, /* CIF       */
	{88, 60},	/* QQSIF     */
	{176, 120}, /* QSIF      */
	{352, 240}, /* SIF       */
	// VGA Resolutions
	{40, 30},	/* QQQQVGA   */
	{80, 60},	/* QQQVGA    */
	{160, 120}, /* QQVGA     */
	{320, 240}, /* QVGA      */
	{640, 480}, /* VGA       */
	{60, 40},	/* HQQQVGA   */
	{120, 80},	/* HQQVGA    */
	{240, 160}, /* HQVGA     */
	// FFT Resolutions
	{64, 32},	/* 64x32     */
	{64, 64},	/* 64x64     */
	{128, 64},	/* 128x64    */
	{128, 128}, /* 128x64    */
	// Other
	{128, 160},	  /* LCD       */
	{128, 160},	  /* QQVGA2    */
	{720, 480},	  /* WVGA      */
	{752, 480},	  /* WVGA2     */
	{800, 600},	  /* SVGA      */
	{1024, 768},  /* XGA       */
	{1280, 1024}, /* SXGA      */
	{1600, 1200}, /* UXGA      */
};

void mt9v034_config(int frame_size_num)
{
	vio_status.cam_frame_size_num = frame_size_num;
	vio_status.cam_frame_size = resolution[frame_size_num][0]*resolution[frame_size_num][1];
	set_framesize(frame_size_num);
}



int IM_MIN(int a, int b)
{
	if (a > b)
		return b;
	else
		return a;
}

#define MICROSECOND_CLKS (1000000)
#define MT9V034_XCLK_FREQ 27000000

static int set_auto_exposure(int enable, int exposure_us)
{
	uint16_t reg, row_time_0, row_time_1;
	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_AEC_AGC_ENABLE, &reg);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_AEC_AGC_ENABLE,
						 (reg & (~MT9V034_AEC_ENABLE)) | ((enable != 0) ? MT9V034_AEC_ENABLE : 0));
	//ret |= sensor->snapshot(sensor, NULL, NULL); // Force shadow mode register to update...

	if ((enable == 0) && (exposure_us >= 0))
	{
		ret |= cambus_readw(MT9V034_SLV_ADDR, MT9V034_WINDOW_WIDTH, &row_time_0);
		ret |= cambus_readw(MT9V034_SLV_ADDR, MT9V034_HORIZONTAL_BLANKING, &row_time_1);

		int exposure = IM_MIN(exposure_us, MICROSECOND_CLKS / 2) * (MT9V034_XCLK_FREQ / MICROSECOND_CLKS);
		int row_time = row_time_0 + row_time_1;
		int coarse_time = exposure / row_time;
		int fine_time = exposure % row_time;

		ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_TOTAL_SHUTTER_WIDTH, coarse_time);
		ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_FINE_SHUTTER_WIDTH_TOTAL, fine_time);
	}
	else if ((enable != 0) && (exposure_us >= 0))
	{
		ret |= cambus_readw(MT9V034_SLV_ADDR, MT9V034_WINDOW_WIDTH, &row_time_0);
		ret |= cambus_readw(MT9V034_SLV_ADDR, MT9V034_HORIZONTAL_BLANKING, &row_time_1);

		int exposure = IM_MIN(exposure_us, MICROSECOND_CLKS / 2) * (MT9V034_XCLK_FREQ / MICROSECOND_CLKS);
		int row_time = row_time_0 + row_time_1;
		int coarse_time = exposure / row_time;

		ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_MAX_EXPOSE, coarse_time);
	}

	return ret;
}

static int set_colorbar(int enable) //是否使能测试条码
{
	uint16_t test;
	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_TEST_PATTERN, &test);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_TEST_PATTERN,
						 (test & (~(MT9V034_TEST_PATTERN_ENABLE | MT9V034_TEST_PATTERN_GRAY_MASK))) | ((enable != 0) ? (MT9V034_TEST_PATTERN_ENABLE | MT9V034_TEST_PATTERN_GRAY_VERTICAL) : 0));

	return ret;
}
static int set_hmirror(int enable) //设置水平对称
{
	uint16_t read_mode;
	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_READ_MODE, &read_mode);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_READ_MODE, // inverted behavior
						 (read_mode & (~MT9V034_READ_MODE_COL_FLIP)) | ((enable == 0) ? MT9V034_READ_MODE_COL_FLIP : 0));

	return ret;
}

static int set_vflip(int enable) //设置竖直对称
{
	uint16_t read_mode;
	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_READ_MODE, &read_mode);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_READ_MODE, // inverted behavior
						 (read_mode & (~MT9V034_READ_MODE_ROW_FLIP)) | ((enable == 0) ? MT9V034_READ_MODE_ROW_FLIP : 0));

	return ret;
}
static int set_framesize(framesize_t framesize)
{
	uint16_t width = resolution[framesize][0];
	uint16_t height = resolution[framesize][1];

	if ((width > MT9V034_MAX_WIDTH) || (height > MT9V034_MAX_HEIGHT))
	{
		return -1;
	}

	uint16_t read_mode;

	if (cambus_readw(MT9V034_SLV_ADDR, MT9V034_READ_MODE, &read_mode) != 0)
	{
		return -1;
	}

	int read_mode_mul = 1;
	read_mode &= 0xFFF0;

	if ((width <= (MT9V034_MAX_WIDTH / 4)) && (height <= (MT9V034_MAX_HEIGHT / 4)))
	{
		read_mode_mul = 4;
		read_mode |= MT9V034_READ_MODE_COL_BIN_4 | MT9V034_READ_MODE_ROW_BIN_4;
	}
	else if ((width <= (MT9V034_MAX_WIDTH / 2)) && (height <= (MT9V034_MAX_HEIGHT / 2)))
	{
		read_mode_mul = 2;
		read_mode |= MT9V034_READ_MODE_COL_BIN_2 | MT9V034_READ_MODE_ROW_BIN_2;
	}

	int ret = 0;

	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_COL_START,
						 ((MT9V034_MAX_WIDTH - (width * read_mode_mul)) / 2) + MT9V034_COL_START_MIN);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_ROW_START,
						 ((MT9V034_MAX_HEIGHT - (height * read_mode_mul)) / 2) + MT9V034_ROW_START_MIN);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_WINDOW_WIDTH, width * read_mode_mul);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_WINDOW_HEIGHT, height * read_mode_mul);

	// Notes: 1. The MT9V034 uses column parallel analog-digital converters, thus short row timing is not possible.
	// The minimum total row time is 690 columns (horizontal width + horizontal blanking). The minimum
	// horizontal blanking is 61. When the window width is set below 627, horizontal blanking
	// must be increased.
	//
	// The STM32H7 needs more than 94+(752-640) clocks between rows otherwise it can't keep up with the pixel rate.
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_HORIZONTAL_BLANKING,
						 MT9V034_HORIZONTAL_BLANKING_DEF + (MT9V034_MAX_WIDTH - IM_MIN(width * read_mode_mul, 640)));

	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_READ_MODE, read_mode);
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_PIXEL_COUNT, (width * height) / 8);

	// We need more setup time for the pixel_clk at the full data rate...
	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_PIXEL_CLOCK, (read_mode_mul == 1) ? MT9V034_PIXEL_CLOCK_INV_PXL_CLK : 0);

	return ret;
}

/**
  * @brief  Changes sensor context based on settings
  */
#if 0
void mt9v034_set_context()
{
	uint16_t new_control;
	if (FLOAT_AS_BOOL(global_data.param[PARAM_VIDEO_ONLY]))
		new_control = 0x8188; // Context B
	else
		new_control = 0x0188; // Context A

	mt9v034_WriteReg16(MTV_CHIP_CONTROL_REG, new_control);
}
#endif
/**
  * @brief  Reads from a specific Camera register
  */
static uint16_t mt9v034_ReadReg16(uint8_t address)
{
	uint8_t tmp[2];
	HAL_StatusTypeDef ret;
#if I2C_GPIO_ENABLE
	i2c_master_read_mem(mt9v034_DEVICE_WRITE_ADDRESS, address, tmp, 2);
#else
	ret = HAL_I2C_Mem_Read(&hi2c1, mt9v034_DEVICE_WRITE_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, tmp, 2, 1000);
#endif
	return tmp[0] << 8 | tmp[1];
}

/**
  * @brief  Writes to a specific Camera register
  */
static uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data)
{
	uint8_t tmp[4];
	int8_t ret;
	/*
	tmp[0] = address;
	tmp[1] = Data >> 8;
	tmp[2] = Data;
*/
	tmp[0] = address;
	tmp[1] = (uint8_t)(Data >> 8);
	tmp[2] = 0xf0;
	tmp[3] = (uint8_t)Data;
#if I2C_GPIO_ENABLE
#if 0
	i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, tmp, 3);
#else
	ret = i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, &tmp[0], 2);
	ret = i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, &tmp[2], 2);
#endif
#else
	ret = HAL_I2C_Master_Transmit(&hi2c1, mt9v034_DEVICE_WRITE_ADDRESS, tmp, 3, 100);
#endif
	return ret;
}