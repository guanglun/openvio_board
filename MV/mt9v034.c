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

extern I2C_HandleTypeDef hi2c1;

struct mt9v034_reg {
	uint8_t addr;
	uint16_t val;
};
#define MT9V034_TABLE_END 0xff
#define MT9V034_TABLE_WAIT_MS 0
enum {
	MT9V034_MODE_640x480,
	MT9V034_MODE_752x480,
};
#define msleep osDelay
//HAL_Delay
static uint16_t mt9v034_ReadReg16(uint8_t address);
static uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data);
static struct mt9v034_reg mt9v034_mode_640x480[] = {
	#if 1
	{0x01, 0x0039},
	{0x02, 0x0004},
	{0x03, 0x01E0},
	{0x04, 0x0280},
	{0x05, 0x005E},
	{0x06, 0x0039},
	{0x07, 0x0388},/* Enable Defective Pixel Correction */
	{0x08, 0x0190},/* Coarse Shutter Width 1 Context A */
	{0x09, 0x01BD},/* Coarse Shutter Width 2 Context A */
	{0x0A, 0x0164},/* Shutter Width Ctrl Context A bit8:Exposure Knee Point */
	{0x0B, 0x01C2},/* Coarse Total Shutter Width Context A */
	{0x0C, 0x0000},
	{0x0D, 0x0330},/* Read Mode Context A */
	#else
	{0x01, 0x0001},
	{0x02, 0x0004},
	{0x03, FULL_IMAGE_ROW_SIZE*4},
	{0x04, FULL_IMAGE_ROW_SIZE*4},
	{0x05, 425+91},
	{0x06, 10},
	//{0x07, 0x0388},/* Enable Defective Pixel Correction */
	{0x08, 443},/* Coarse Shutter Width 1 Context A */
	{0x09, 473},/* Coarse Shutter Width 2 Context A */
	{0x0A, 0x0164},/* Shutter Width Ctrl Context A bit8:Exposure Knee Point */
	{0x0B, 480},/* Coarse Total Shutter Width Context A */
	//{0x0C, 0x0000},
	{0x0D, 0x030a},/* Read Mode Context A */
	#endif
	{0x0E, 0x0000},
	{0x0F, 0x0100},/* Sensor Type Control bit0:High Dynamic Range */
	{0x10, 0x0040},
	{0x11, 0x8042},
	{0x12, 0x0022},
	{0x13, 0x2D2E},
	{0x14, 0x0E02},
	{0x15, 0x0E32},
	{0x16, 0x2802},
	{0x17, 0x3E38},
	{0x18, 0x3E38},
	{0x19, 0x2802},
	{0x1A, 0x0428},
	{0x1B, 0x0000},
	{0x1C, 0x0303},	/* ADC Companding Mode */
	{0x1D, 0x0040},
	{0x1E, 0x0000},
	{0x1F, 0x0000},
	{0x20, 0x03C7},
	{0x21, 0x0020},
	{0x22, 0x0020},
	{0x23, 0x0010},
	{0x24, 0x001B},
	{0x25, 0x001A},
	{0x26, 0x0004},
	{0x27, 0x000C},
	{0x28, 0x0010},
	{0x29, 0x0010},
	{0x2A, 0x0020},
	{0x2B, 0x0003},
	{0x2C, 0x0004},
	{0x2D, 0x0004},
	{0x2E, 0x0007},
	{0x2F, 0x0003},
	{0x30, 0x0003},
	{0x31, 0x001F},
	{0x32, 0x001A},
	{0x33, 0x0012},
	{0x34, 0x0003},
	{0x35, 0x0028},/* Analog Gain Context A bit0:6 */
	{0x36, 0x0028},/* Analog Gain Context B */
	{0x37, 0x0000},
	{0x38, 0x0000},
	{0x39, 0x0025},
	{0x3A, 0x0020},
	{0x3B, 0x0003},
	{0x3C, 0x0003},
	{0x46, 0x231D},
	{0x47, 0x0080},
	{0x4C, 0x0002},
	{0x70, 0x0000},
	{0x71, 0x002A},
	{0x72, 0x0000},
	{0x7F, 0x0000},
	{0x80, 0x04F4},
	{0x81, 0x04F4},
	{0x82, 0x04F4},
	{0x83, 0x04F4},
	{0x84, 0x04F4},
	{0x85, 0x04F4},
	{0x86, 0x04F4},
	{0x87, 0x04F4},
	{0x88, 0x04F4},
	{0x89, 0x04F4},
	{0x8A, 0x04F4},
	{0x8B, 0x04F4},
	{0x8C, 0x04F4},
	{0x8D, 0x04F4},
	{0x8E, 0x04F4},
	{0x8F, 0x04F4},
	{0x90, 0x04F4},
	{0x91, 0x04F4},
	{0x92, 0x04F4},
	{0x93, 0x04F4},
	{0x94, 0x04F4},
	{0x95, 0x04F4},
	{0x96, 0x04F4},
	{0x97, 0x04F4},
	{0x98, 0x04F4},
	{0x99, 0x0000},
	{0x9A, 0x0096},
	{0x9B, 0x012C},
	{0x9C, 0x01C2},
	{0x9D, 0x0258},
	{0x9E, 0x02F0},
	{0x9F, 0x0000},
	{0xA0, 0x0060},
	{0xA1, 0x00C0},
	{0xA2, 0x0120},
	{0xA3, 0x0180},
	{0xA4, 0x01E0},
	{0xA5, 0x003A},
	{0xA6, 0x0002},
	{0xA8, 0x0000},
	{0xA9, 0x0002},
	{0xAA, 0x0002},
	{0xAB, 0x0040},	/* Analog Gain Max */
	{0xAC, 0x0001},
	{0xAD, 0x01E0},
	{0xAE, 0x0014},
	{0xAF, 0x0003},	/* AEC, AGC */
	{0xB0, 0xABE0},
	{0xB1, 0x0002},
	{0xB2, 0x0010},
	{0xB3, 0x0010},
	{0xB4, 0x0000},
	{0xB5, 0x0000},
	{0xB6, 0x0000},
	{0xB7, 0x0000},
	{0xBF, 0x0016},
	{0xC0, 0x000A},
	{0xC2, 0x18D0},
	{0xC3, 0x007F},
	{0xC4, 0x007F},
	{0xC5, 0x007F},
	{0xC6, 0x0000},
	{0xC7, 0x4416},
	{0xC8, 0x4421},
	{0xC9, 0x0002},
	{0xCA, 0x0004},
	{0xCB, 0x01E0},
	{0xCC, 0x02EE},
	{0xCD, 0x0100},
	{0xCE, 0x0100},
	{0xCF, 0x0190},
	{0xD0, 0x01BD},
	{0xD1, 0x0064},
	{0xD2, 0x01C2},
	{0xD3, 0x0000},
	{0xD4, 0x0000},
	{0xD5, 0x0000},
	{0xD6, 0x0000},
	{0xD7, 0x0000},
	{0xD8, 0x0000},
	{0xD9, 0x0000},
	{MT9V034_TABLE_END, 0x0000}
};
int reset_mt9v034_by_hardware(void)
{
#if 0
	//standby pin must be low
	HAL_GPIO_WritePin(GPIOE, mono_standby_Pin, SET);
	//HAL_GPIO_WritePin(GPIOE, mono_reset_bar_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOE, mono_standby_Pin, RESET);
	HAL_GPIO_WritePin(GPIOE, mono_reset_bar_Pin, RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOE, mono_reset_bar_Pin, SET);
	HAL_Delay(10);
#endif
	return 0;
}
static int mt9v034_write_table(		const struct mt9v034_reg table[])
{
	const struct mt9v034_reg *next;
	int err = 0;
	uint16_t val;

	for (next = table; next->addr != MT9V034_TABLE_END; next++) {

		if (next->addr == MT9V034_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}
		HAL_Delay(1);
		val = next->val;

		err = mt9v034_WriteReg16(next->addr, val);

		if (err != 0) {
			printf("[mt9v034]: write failed\r\n");
		}
	}

	return err;
}

int mt9v034_start_stream(void)
{
	uint16_t chip_version = 0;
	int retry_num = 1, err = 0;

	/* MT9V034 Soft Reset */
	/* Reset */
	//mt9v034_WriteReg16(MTV_SOFT_RESET_REG, 0x01);
	//msleep(1);

	while (!chip_version && retry_num-- > 0)
		chip_version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);

	if (chip_version != 0x1324) {
		printf("read err\r\n");
		//while(1);
		return -1;
	}
	printf("[mt9v034]: ok\r\n");




		err = mt9v034_write_table(mt9v034_mode_640x480);
	/* Reset */
		//mt9v034_WriteReg16(MTV_SOFT_RESET_REG, 0x01);
	return err;
}
#if 0
/**
  * @brief  Configures the mt9v034 camera with two context (binning 4 and binning 2).
  */
void mt9v034_context_configuration(void)
{
	/* Chip Control
	 *
	 * bits           | 15 | ... | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * -------------------------------------------------------------------
	 * current values | 0  | ... | 0 | 1 | 1 | 0 | 0 | 0 | 1 | 0 | 0 | 0 |
	 *
	 * (0:2) Scan Mode (Progressive scan)
	 * (3:4) Sensor Operation Mode (Master mode)
	 * (5) Stereoscopy Mode (Disable)
	 * (6) Stereoscopic Master/Slave mode (not used)
	 * (7) Parallel Output Enable (Enable)
	 * (8) Simultaneous/Sequential Mode (Simultaneous mode)
	 * (9) Reserved
	 *
	 * (15)Context A (0) / Context B (1)
	 *
	 */

	uint16_t new_control;

	if (FLOAT_AS_BOOL(global_data.param[PARAM_VIDEO_ONLY]))
		new_control = 0x8188; // Context B
	else
		new_control = 0x0188; // Context A

	/* image dimentions */
	uint16_t new_width_context_a  = global_data.param[PARAM_IMAGE_WIDTH] * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_a = global_data.param[PARAM_IMAGE_HEIGHT] * 4;
	uint16_t new_width_context_b  = FULL_IMAGE_ROW_SIZE * 4; // windowing off, row + col bin reduce size
	uint16_t new_height_context_b = FULL_IMAGE_COLUMN_SIZE * 4;

	/* blanking settings */
	uint16_t new_hor_blanking_context_a = 425 + MINIMUM_HORIZONTAL_BLANKING;// 709 is minimum value without distortions
	uint16_t new_ver_blanking_context_a = 10; // this value is the first without image errors (dark lines)
	uint16_t new_hor_blanking_context_b = MAX_IMAGE_WIDTH - new_width_context_b + MINIMUM_HORIZONTAL_BLANKING;
	if (new_hor_blanking_context_b < 800) {
		new_hor_blanking_context_b = 800;
	}
	uint16_t new_ver_blanking_context_b = 10;


	/* Read Mode
	 *
	 * bits           | ... | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 * -------------------------------------------------------------------
	 * current values | ... |  0 | 1 | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 0 |
	 *
	 * (1:0) Row Bin
	 * (3:2) Column Bin
	 * (9:8) Reserved
	 *
	 */
	uint16_t new_readmode_context_a = 0x30A ; // row + col bin 4 enable, (9:8) default
	uint16_t new_readmode_context_b = 0x305 ; // row bin 2 col bin 4 enable, (9:8) default

	/*
	 * Settings for both context:
	 *
	 * Exposure time should not affect frame time
	 * so we set max on 64 (lines) = 0x40
	 */
	uint16_t min_exposure = 0x0001; // default
	uint16_t pixel_count = 4096; //64x64 take all pixels to estimate exposure time // VALID RANGE: 1-65535
	uint16_t shutter_width_ctrl = 0x0164; // default from context A
	uint16_t aec_update_freq = 0x02; // default Number of frames to skip between changes in AEC VALID RANGE: 0-15
	uint16_t aec_low_pass = 0x01; // default VALID RANGE: 0-2
	uint16_t agc_update_freq = 0x02; // default Number of frames to skip between changes in AGC VALID RANGE: 0-15
	uint16_t agc_low_pass = 0x02; // default VALID RANGE: 0-2

	uint16_t resolution_ctrl = 0x0303; // 12 bit adc for low light
	uint16_t max_gain = global_data.param[PARAM_GAIN_MAX];
	uint16_t max_exposure = global_data.param[PARAM_EXPOSURE_MAX];
	uint16_t coarse_sw1 = global_data.param[PARAM_SHTR_W_1];
	uint16_t coarse_sw2 = global_data.param[PARAM_SHTR_W_2];
	uint16_t total_shutter_width = global_data.param[PARAM_SHTR_W_TOT];
	uint16_t hdr_enabled = 0x0000;
	bool hdr_enable_flag = global_data.param[PARAM_HDR] > 0;
	if (hdr_enable_flag) {
		hdr_enabled = 0x0100;
	}

	bool aec_enable_flag = global_data.param[PARAM_AEC] > 0;
	uint16_t aec_agc_enabled = 0x0000;
	if (aec_enable_flag) {
		aec_agc_enabled |= (1 << 0);
	}

	bool agc_enable_flag = global_data.param[PARAM_AGC] > 0;
	if (agc_enable_flag) {
		aec_agc_enabled |= (1 << 1);
	}

	uint16_t desired_brightness = global_data.param[PARAM_BRIGHT];

	uint16_t row_noise_correction = 0x0000; // default
	uint16_t test_data = 0x0000; // default

	if(FLOAT_AS_BOOL(global_data.param[PARAM_IMAGE_ROW_NOISE_CORR]) && !FLOAT_AS_BOOL(global_data.param[PARAM_IMAGE_TEST_PATTERN]))
		row_noise_correction = 0x0101;
	else
		row_noise_correction = 0x0000;

	if (FLOAT_AS_BOOL(global_data.param[PARAM_IMAGE_TEST_PATTERN]))
		test_data = 0x3000; //enable vertical gray shade pattern
	else
		test_data = 0x0000;
	//读取版本号,默认是0x1324
	uint16_t version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);
	if (version == 0x1324)
	{
		mt9v034_WriteReg16(MTV_CHIP_CONTROL_REG, new_control);

		// Initialize frame control reg
		/*
			该寄存器主要涉及到hsync vsync pclk的时钟极性 配置为hsync高有效 vsync高有效 pclk下降沿
		*/
		mt9v034_WriteReg(0x72, 0x0000);

		// Write reserved registers per Rev G datasheet table 8 recommendations
		mt9v034_WriteReg16(0x13, 0x2D2E);
		mt9v034_WriteReg16(0x20, 0x03C7);
		mt9v034_WriteReg16(0x24, 0x001B);
		mt9v034_WriteReg16(0x2B, 0x0003);
		mt9v034_WriteReg16(0x2F, 0x0003);

		/* Context A */
		#if 0
		mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_A, new_width_context_a);
		mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_A, new_height_context_a);
		mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_A, new_hor_blanking_context_a);
		mt9v034_WriteReg16(MTV_VER_BLANKING_REG_A, new_ver_blanking_context_a);
		mt9v034_WriteReg16(MTV_READ_MODE_REG_A, new_readmode_context_a);
		#if 1
		mt9v034_WriteReg16(MTV_COLUMN_START_REG_A, (MAX_IMAGE_WIDTH - new_width_context_a) / 2 + MINIMUM_COLUMN_START); // Set column/row start point for lower resolutions (center window)
		mt9v034_WriteReg16(MTV_ROW_START_REG_A, (MAX_IMAGE_HEIGHT - new_height_context_a) / 2 + MINIMUM_ROW_START);
		#else
		mt9v034_WriteReg16(MTV_COLUMN_START_REG_A, MINIMUM_COLUMN_START); // default
		mt9v034_WriteReg16(MTV_ROW_START_REG_A, MINIMUM_ROW_START);
		#endif
		mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_A, coarse_sw1);
		mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_A, coarse_sw2);
		mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_A, shutter_width_ctrl);
		mt9v034_WriteReg16(MTV_COARSE_SW_TOTAL_REG_A, total_shutter_width);
		#else
		mt9v034_WriteReg16(0x04, 64*4);
		mt9v034_WriteReg16(0x03, 64*4);
		mt9v034_WriteReg16(0x05, 425+91);
		mt9v034_WriteReg16(0x06, 10);
		mt9v034_WriteReg16(0x0d, 0x30a);
		#if 0
		mt9v034_WriteReg16(MTV_COLUMN_START_REG_A, (MAX_IMAGE_WIDTH - new_width_context_a) / 2 + MINIMUM_COLUMN_START); // Set column/row start point for lower resolutions (center window)
		mt9v034_WriteReg16(MTV_ROW_START_REG_A, (MAX_IMAGE_HEIGHT - new_height_context_a) / 2 + MINIMUM_ROW_START);
		#else
		mt9v034_WriteReg16(0x01, 1); // default
		mt9v034_WriteReg16(0x02, 4);
		#endif
		mt9v034_WriteReg16(0x08, 443);
		mt9v034_WriteReg16(0x09, 473);
		mt9v034_WriteReg16(0x0a, 0x0164);
		mt9v034_WriteReg16(0x0b, 480);
		#endif

		/* Context B */
		mt9v034_WriteReg16(MTV_WINDOW_WIDTH_REG_B, new_width_context_b);
		mt9v034_WriteReg16(MTV_WINDOW_HEIGHT_REG_B, new_height_context_b);
		mt9v034_WriteReg16(MTV_HOR_BLANKING_REG_B, new_hor_blanking_context_b);
		mt9v034_WriteReg16(MTV_VER_BLANKING_REG_B, new_ver_blanking_context_b);
		mt9v034_WriteReg16(MTV_READ_MODE_REG_B, new_readmode_context_b);
		mt9v034_WriteReg16(MTV_COLUMN_START_REG_B, MINIMUM_COLUMN_START); // default
		mt9v034_WriteReg16(MTV_ROW_START_REG_B, MINIMUM_ROW_START);
		mt9v034_WriteReg16(MTV_COARSE_SW_1_REG_B, coarse_sw1);
		mt9v034_WriteReg16(MTV_COARSE_SW_2_REG_B, coarse_sw2);
		mt9v034_WriteReg16(MTV_COARSE_SW_CTRL_REG_B, shutter_width_ctrl);
		mt9v034_WriteReg16(MTV_COARSE_SW_TOTAL_REG_B, total_shutter_width);

		/* General Settings */
		mt9v034_WriteReg16(MTV_ROW_NOISE_CORR_CTRL_REG, row_noise_correction);
		mt9v034_WriteReg16(MTV_AEC_AGC_ENABLE_REG, aec_agc_enabled); // disable AEC/AGC for both contexts
		mt9v034_WriteReg16(MTV_HDR_ENABLE_REG, hdr_enabled); // disable HDR on both contexts
		mt9v034_WriteReg16(MTV_MIN_EXPOSURE_REG, min_exposure);
		mt9v034_WriteReg16(MTV_MAX_EXPOSURE_REG, max_exposure);
		mt9v034_WriteReg16(MTV_MAX_GAIN_REG, max_gain);
		mt9v034_WriteReg16(MTV_AGC_AEC_PIXEL_COUNT_REG, pixel_count);
		mt9v034_WriteReg16(MTV_AGC_AEC_DESIRED_BIN_REG, desired_brightness);
		mt9v034_WriteReg16(MTV_ADC_RES_CTRL_REG, resolution_ctrl); // here is the way to regulate darkness :)

		mt9v034_WriteReg16(MTV_DIGITAL_TEST_REG, test_data);//enable test pattern

		mt9v034_WriteReg16(MTV_AEC_UPDATE_REG,aec_update_freq);
		mt9v034_WriteReg16(MTV_AEC_LOWPASS_REG,aec_low_pass);
		mt9v034_WriteReg16(MTV_AGC_UPDATE_REG,agc_update_freq);
		mt9v034_WriteReg16(MTV_AGC_LOWPASS_REG,agc_low_pass);

		/* Reset */
		mt9v034_WriteReg16(MTV_SOFT_RESET_REG, 0x01);
	}

}
#endif
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
	tmp[1] = (uint8_t)( Data >> 8);
	tmp[2] = 0xf0;
	tmp[3] = (uint8_t) Data;
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

