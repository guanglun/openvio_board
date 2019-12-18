#include "mpu6000.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "spi.h"

extern SPI_HandleTypeDef hspi3;

#define MPU6000_TIMEOUT_VALUE 0xFFFF

void mpu6000_init(void)
{
    while(1)
    {
        uint8_t recv[6],cmd[6];
        //HAL_GPIO_TogglePin(GPIOD, IMU_SPI_CS_Pin);
        HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_SET);
        
        cmd[0] = 0x6B;
        cmd[1] = 0x03;
        
        HAL_SPI_Transmit(&hspi3, cmd, 2, MPU6000_TIMEOUT_VALUE); 
        
        HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
        
        cmd[0] = 0x75 | 0x80;
        HAL_SPI_Transmit(&hspi3, cmd, 1, MPU6000_TIMEOUT_VALUE); 
        
        
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
        }
        
        //HAL_GPIO_WritePin(GPIOD, IMU_SPI_CS_Pin, GPIO_PIN_RESET);
        
        osDelay(1000);
    }
}
