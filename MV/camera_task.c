#include "camera_task.h"
#include "cambus.h"
#include "mt9v034.h"
#include "dcmi.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "mpu6000.h"

#include "ff.h"

FATFS fs;
FRESULT fr;
FIL fd;

extern USBD_HandleTypeDef hUsbDeviceHS;
extern int frame_count;
extern DMA_BUFFER uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];
extern int line_cnt;

DMA_BUFFER uint8_t mpu6000_data[14];
char is_camera_run = 0;

static void camera_img_send(void);

#define LIBUSB_REQUEST_CAMERA_START 0xA0
#define LIBUSB_REQUEST_CAMERA_STOP 0xA1

uint8_t camera_ctrl(USBD_SetupReqTypedef *req)
{
	uint8_t result = 'F';
	switch (req->bRequest)
	{
	case LIBUSB_REQUEST_CAMERA_START:
		is_camera_run = 1;
		result = 'S';
		break;
	case LIBUSB_REQUEST_CAMERA_STOP:
		is_camera_run = 0;
		result = 'S';
		break;
	default:
		break;
	}

	return result;
}

void StartCameraTask(void const *argument)
{

	//
	//mpu6000_init();

	//	char filename[] = "test.txt";
	//	uint8_t write_dat[] = "	char hello[] = "hello";
	//
	//	while(1)
	//	{
	//		while(CDC_Transmit_HS(hello,sizeof(hello)) == USBD_BUSY);
	//
	//		osDelay(1000);
	//	}Hello,FATFS!\n";
	//	uint16_t write_num = 0;
	//
	//	fr = f_mount(&fs, "", 0);
	//	if(fr == FR_OK)
	//	{
	//		printf("SD card mount ok!\r\n");
	//	}
	//	else
	//	{
	//		printf("SD card mount error, error code:%d.\r\n",fr);
	//	}
	//
	//	fr = f_open(&fd, filename, FA_CREATE_ALWAYS | FA_WRITE);
	//	if(fr == FR_OK)
	//	{
	//		printf("open file \"%s\" ok! \r\n", filename);
	//	}
	//	else
	//	{
	//	printf("open file \"%s\" error : %d\r\n", filename, fr);
	//	}

	//	fr = f_write(&fd, write_dat, sizeof(write_dat), (void *)&write_num);
	//	if(fr == FR_OK)
	//	{
	//		printf("write %d dat to file \"%s\" ok,dat is \"%s\".\r\n", write_num, filename, write_dat);
	//	}
	//	else
	//	{
	//		printf("write dat to file \"%s\" error,error code is:%d\r\n", filename, fr);
	//	}

	//	fr = f_close(&fd);
	//	if(fr == FR_OK)
	//	{
	//		printf("close file \"%s\" ok!\r\n", filename);
	//	}
	//	else
	//	{
	//		printf("close file \"%s\" error, error code is:%d.\r\n", filename, fr);
	//	}

	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	int ret = cambus_scan();

	printf("cambus_scan %02X\r\n", ret);

	mt9v034_init();
	osDelay(10);

	int count = 5;

	//	while (count--)
	//	{
	//		osDelay(1000);
	//		printf("count %d\r\n", count);
	//	}
	
	while (1)
	{
		//CDC_Transmit_HS(tx_buf,12);
		mpu6000_read(mpu6000_data);
		//MPU_Transmit_HS(mpu6000_data,14);
		
		for(int i=0;i<14;i++)
		{
			printf("%02X ",mpu6000_data[i]);
		}
		printf("\r\n");
//		printf("%d %d %d %d %d %d %d\r\n",	(short int)mpu6000_data[0],
//											(short int)mpu6000_data+2,
//											(short int)mpu6000_data+4,	
//											(short int)mpu6000_data+6,
//											(short int)mpu6000_data+8,
//											(short int)mpu6000_data+10,
//											(short int)mpu6000_data+12
//											);
											
		osDelay(1000);
	}
	dcmi_dma_start();

	while (1)
	{
		while (!is_camera_run)
		{
			osDelay(10);
		}

		frame_count++;
		camera_img_send();

		dcmi_dma_start();
	}
}

void camera_img_send(void)
{
#define SNED_SIZE (63 * 1024)

	for (uint32_t i = 0; i < FULL_IMAGE_SIZE; i += SNED_SIZE)
	{
		if (i + SNED_SIZE > FULL_IMAGE_SIZE)
		{
			while (CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i], FULL_IMAGE_SIZE - i) == USBD_BUSY)
				;
		}
		else
		{
			while (CDC_Transmit_HS(&dcmi_image_buffer_8bit_1[i], SNED_SIZE) == USBD_BUSY)
				;
		}
	}

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
	while (hcdc->TxState != 0)
		;
}
