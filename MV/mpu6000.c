#include "mpu6000.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "spi.h"

extern SPI_HandleTypeDef hspi3;

#define MPU6000_ENABLE() 	HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET)
#define MPU6000_DISABLE() 	HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET)

#define MPU6000_TIMEOUT_VALUE 0xFF

void mpu6000_write_reg(uint8_t reg,uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = value;
	
	MPU6000_ENABLE();

    if(HAL_SPI_Transmit(&hspi3, cmd, 2, MPU6000_TIMEOUT_VALUE) != HAL_OK) 
	{
		printf("mpu6000_write_reg fail");
	}
	
	MPU6000_DISABLE();
        
    osDelay(10);
}

int mpu6000_init(void)
{

        uint8_t recv[6],cmd[6];
	
		MPU6000_ENABLE();
        
		cmd[0] = (MPU6000_RA_WHO_AM_I | 0x80);  //0x68
	
	    if(HAL_SPI_Transmit(&hspi3, cmd, 1, MPU6000_TIMEOUT_VALUE) != HAL_OK) 
		{

		}
	
        if (HAL_SPI_Receive(&hspi3, recv,1,MPU6000_TIMEOUT_VALUE) == HAL_OK)
        {
            
            printf("MPU6000 RECV:");
            for(int i=0;i<1;i++)
            {
                printf("%02X ",recv[i]);
            }
            printf("\r\n");
            
        }else{
            printf("MPU6000 ID ERROR\r\n");
			return 1;
        }
        
        MPU6000_DISABLE();
		
		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_1,		0x80);
		osDelay(100);
		mpu6000_write_reg(MPU6000_RA_SIGNAL_PATH_RESET,	0x07);
		osDelay(100);
		
		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_1,	0x01);
		mpu6000_write_reg(MPU6000_RA_PWR_MGMT_2,	0x00);
		mpu6000_write_reg(MPU6000_RA_SMPLRT_DIV,	0x00);
		mpu6000_write_reg(MPU6000_RA_USER_CTRL,		0x10);
		mpu6000_write_reg(MPU6000_RA_CONFIG,		0x02);
		mpu6000_write_reg(MPU6000_RA_GYRO_CONFIG,	0x18);
		mpu6000_write_reg(MPU6000_RA_ACCEL_CONFIG,	0x10);
		

        
        return 0;
    
}

void mpu6000_read(uint8_t *buf)
{
        uint8_t cmd[2];
		
        MPU6000_ENABLE();
        
        cmd[0] = (MPU6000_RA_ACCEL_XOUT_H | 0x80);
        HAL_SPI_Transmit(&hspi3, cmd, 1, MPU6000_TIMEOUT_VALUE); 
        
        
        if (HAL_SPI_Receive(&hspi3, buf,1,MPU6000_TIMEOUT_VALUE) != HAL_OK)
 		{
			printf("mpu6000_read\r\n");
		}		
		
		MPU6000_DISABLE();		
}
