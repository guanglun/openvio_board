#include "camera.h"

#include "openvio.h"
#include "mt9v034.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

DMA_BUFFER uint8_t dcmi_image_buffer[FULL_IMAGE_SIZE] = {0};
int frame_count = 0;
int line_cnt = 0,count = 0,start=0;

void dcmi_dma_start(void)
{

  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_LINE);

  HAL_DCMI_Stop(&hdcmi); 
  line_cnt = 0;
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dcmi_image_buffer, vio_status.cam_frame_size/4);
  line_cnt = 0;
  __HAL_DCMI_ENABLE(&hdcmi);

  while ((DCMI->CR & DCMI_CR_CAPTURE) != 0)
  {
	  icm20948_transmit();
  }
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{

}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{

}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{

}