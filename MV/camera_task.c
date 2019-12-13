#include "camera_task.h"
#include "cambus.h"
#include "mt9v034.h"
#include "dcmi.h"
#include "usbd_def.h"

extern int frame_count;
extern DMA_BUFFER uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE];

void StartCameraTask(void const * argument)
{
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
	osDelay(10);
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
	osDelay(10);
	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
	osDelay(10);
	
	int ret = cambus_scan();

	printf("cambus_scan %02X\r\n", ret);

	mt9v034_init();

	dcmi_dma_start();
	
	while(1)
	{
		//printf("%d\r\n",frame_count);
		//frame_count = 0;
		//osDelay(1000);
		//if(frame_count > 0)
		{
			printf("send start\r\n");
			frame_count = 0;
			
			
			for(uint32_t i = 0;i<FULL_IMAGE_SIZE;i += 0xFFFF)
			{
				if( i + 0xFFFF > FULL_IMAGE_SIZE)
				{
					while(CDC_Transmit_HS(dcmi_image_buffer_8bit_1,FULL_IMAGE_SIZE - i) == USBD_BUSY);
				}else{
					while(CDC_Transmit_HS(dcmi_image_buffer_8bit_1,0xFFFF) == USBD_BUSY);
				}
					
			}
			
			
			printf("send end\r\n");
			
			
			osDelay(5000);
			dcmi_dma_start();
		}
		
	}
}
