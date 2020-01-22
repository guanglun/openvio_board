#include "icm20948.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "openvio_def.h"
#include "spi.h"

#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "openvio.h"

extern struct OPENVIO_STATUS vio_status;

extern SPI_HandleTypeDef hspi2;

extern USBD_HandleTypeDef hUsbDeviceHS;
osThreadId IMUTaskHandle;
SemaphoreHandle_t xIMUSemaphore;

#define ICM20948_ENABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET)
#define ICM20948_DISABLE() HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET)

#define ICM20948_TIMEOUT_VALUE 0xFF

static int icm20948_read_one_reg(uint8_t reg, uint8_t *buf)
{
	uint8_t cmd = reg | 0x80;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	if (HAL_SPI_Receive(&hspi2, buf, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	ICM20948_DISABLE();

	return 0;
}

static int icm20948_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t cmd = reg | 0x80;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, &cmd, 1, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	if (HAL_SPI_Receive(&hspi2, buf, len, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	ICM20948_DISABLE();

	return 0;
}

void StartIMUTask(void const *argument)
{
	for (;;)
	{
		if (xSemaphoreTake(xIMUSemaphore, portMAX_DELAY) == pdTRUE)
		{
			icm20948_transmit();
		}
	}
}

static int icm20948_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = reg & 0x7F;
	cmd[1] = value;

	ICM20948_ENABLE();

	if (HAL_SPI_Transmit(&hspi2, cmd, 2, ICM20948_TIMEOUT_VALUE) != HAL_OK)
	{
		return 1;
	}

	ICM20948_DISABLE();
	osDelay(10);

	return 0;
}

void icm_mag_write(uint8_t reg, uint8_t value)
{
	icm20948_write_reg(0x7F, 0x30);

	osDelay(1);
	icm20948_write_reg(0x03, 0x0C); //mode: write

	osDelay(1);
	icm20948_write_reg(0x04, reg); //set reg addr

	osDelay(1);
	icm20948_write_reg(0x06, value); //send value

	osDelay(1);
}

static uint8_t icm_mag_read(uint8_t reg)
{
	uint8_t Data;
	icm20948_write_reg(0x7F, 0x30);
	osDelay(1);
	icm20948_write_reg(0x03, 0x0C | 0x80);
	osDelay(1);
	icm20948_write_reg(0x04, reg); // set reg addr
	osDelay(1);
	icm20948_write_reg(0x06, 0xff); //read
	osDelay(1);
	icm20948_write_reg(0x7F, 0x00);

	icm20948_read_reg(0x3B, &Data, 1);
	osDelay(1);
	return Data;
}

void icm20948_read_mag(int16_t magn[3])
{
	uint8_t mag_buffer[10];

	mag_buffer[0] = icm_mag_read(0x01);

	mag_buffer[1] = icm_mag_read(0x11);
	mag_buffer[2] = icm_mag_read(0x12);
	magn[0] = mag_buffer[1] | mag_buffer[2] << 8;
	mag_buffer[3] = icm_mag_read(0x13);
	mag_buffer[4] = icm_mag_read(0x14);
	magn[1] = mag_buffer[3] | mag_buffer[4] << 8;
	mag_buffer[5] = icm_mag_read(0x15);
	mag_buffer[6] = icm_mag_read(0x16);
	magn[2] = mag_buffer[5] | mag_buffer[6] << 8;

	icm_mag_write(0x31, 0x01);
}

void ICM_ReadAccelGyroData(int16_t *accel_data, int16_t *gyro_data)
{
	uint8_t raw_data[12];
	icm20948_read_reg(0x2D, raw_data, 12);

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

	// accel_data[0] = accel_data[0] / 8;
	// accel_data[1] = accel_data[1] / 8;
	// accel_data[2] = accel_data[2] / 8;

	// gyro_data[0] = gyro_data[0] / 250;
	// gyro_data[1] = gyro_data[1] / 250;
	// gyro_data[2] = gyro_data[2] / 250;
}
void ICM_SelectBank(uint8_t bank)
{
	icm20948_write_reg(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void)
{
	icm20948_write_reg(0x03, 0x78);
}
void ICM_SetClock(uint8_t clk)
{
	icm20948_write_reg(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void)
{
	icm20948_write_reg(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM_AccelGyroOn(void)
{
	icm20948_write_reg(0x07, (0x00 | 0x00));
}
uint8_t icm_who_am_i(void)
{
	uint8_t spiData = 0x01;
	icm20948_read_one_reg(0x00, &spiData);
	return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf)
{
	icm20948_write_reg(GYRO_CONFIG_1, (rate | lpf));
}

int icm20948_init(void)
{
	uint8_t id = icm_who_am_i();
	printf("[ICM-20948] [ID: %02X]\r\n", id);
	if (id == 0xEA)
	{
		printf("[ICM-20948] [Init Success]\r\n");
		ICM_SelectBank(USER_BANK_0);
		osDelay(10);
		ICM_Disable_I2C();
		osDelay(10);
		ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
		osDelay(10);
		ICM_AccelGyroOff();
		osDelay(20);
		ICM_AccelGyroOn();
		osDelay(10);

		ICM_SelectBank(USER_BANK_2);
		osDelay(20);
		ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
		osDelay(10);

		// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
		icm20948_write_reg(0x00, 0x0A);
		osDelay(10);

		// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
		icm20948_write_reg(0x14, (0x04 | 0x11));

		// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
		icm20948_write_reg(0x10, 0x00);
		osDelay(10);

		// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
		icm20948_write_reg(0x11, 0x0A);
		osDelay(10);

		ICM_SelectBank(USER_BANK_2);
		osDelay(20);

		// Configure AUX_I2C Magnetometer (onboard ICM-20948)
		icm20948_write_reg(0x7F, 0x00);
		icm20948_write_reg(0x0F, 0x30);
		icm20948_write_reg(0x03, 0x20);
		icm20948_write_reg(0x7F, 0x30);
		icm20948_write_reg(0x01, 0x4D);
		icm20948_write_reg(0x02, 0x01);
		icm20948_write_reg(0x05, 0x81);
		icm_mag_write(0x32, 0x01);
		osDelay(1000);
		icm_mag_write(0x31, 0x02);
	}
	else
	{
		printf("[ICM-20948] [Init Fail]\r\n");
	}

	/* 创建信号量 */
	//    xIMUSemaphore = xSemaphoreCreateBinary();
	//    osThreadDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 512);
	//    IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

//	while (1)
//	{
//		static uint8_t icm20948_data[14];
//		static short acc[3], gyro[3], mag[2],temp;
//		ICM_SelectBank(USER_BANK_0);
//		ICM_ReadAccelGyroData(acc, gyro);
//		

//		//ICM_SelectBank(USER_BANK_2);
//		icm20948_read_mag(mag);

//		printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
//		osDelay(10);
//	}

	return 0;
}

void icm20948_read(uint8_t *buf)
{
	//icm20948_read_reg((MPU6000_RA_ACCEL_XOUT_H | 0x80), buf, 14);
}

void icm20948_transmit(void)
{
	// static uint8_t icm20948_data[14];
	// static short acc[3], gyro[3], temp;
	// USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

	// if(vio_status.is_imu_send)
	// {
	// 	vio_status.is_imu_send = 0;
	// 	icm20948_read(mpu6000_data);
	// 	//MPU_Transmit_HS(mpu6000_data, 14);

	// 	openvio_usb_send(SENSOR_USB_IMU,icm20948_data, 14);

	// }

	// ICM_ReadAccelGyroData(acc, gyro);
	// printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
	// for(int i=0;i<14;i++)
	// {
	// 	printf("%02X ",mpu6000_data[i]);
	// }
	// printf("\r\n");

	//	acc[0] = (short)((mpu6000_data[0]<<8)|mpu6000_data[1]);
	//	acc[1] = (short)((mpu6000_data[2]<<8)|mpu6000_data[3]);
	//	acc[2] = (short)((mpu6000_data[4]<<8)|mpu6000_data[5]);
	//
	//	temp = (short)((mpu6000_data[6]<<8)|mpu6000_data[7]);
	//
	//	gyro[0] = (short)((mpu6000_data[8]<<8)|mpu6000_data[9]);
	//	gyro[1] = (short)((mpu6000_data[10]<<8)|mpu6000_data[11]);
	//	gyro[2] = (short)((mpu6000_data[12]<<8)|mpu6000_data[13]);
	//
	//	printf("%d %d %d %d %d %d %d\r\n", 	acc[0],acc[1],acc[2],
	//										temp,
	//										gyro[0],gyro[1],gyro[2]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static BaseType_t xHigherPriorityTaskWoken;
	if (vio_status.imu_status == SENSOR_STATUS_START)
	{
		vio_status.is_imu_send = 1;
		//		xSemaphoreGiveFromISR( xIMUSemaphore, &xHigherPriorityTaskWoken );
		//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}