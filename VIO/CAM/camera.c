#include "camera.h"

#include "openvio.h"
#include "mt9v034.h"
#include "ov7725.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

DMA_BUFFER uint8_t dcmi_image_buffer[FULL_IMAGE_SIZE/2] = {0};

int frame_count = 0;
int line_cnt = 0,count = 0,start=0;

void camera_init(void)
{
	uint8_t chip_id,cam_slv_addr;

	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	cam_slv_addr = cambus_scan();
    
	printf("[cam slv addr][%02X]\r\n", cam_slv_addr);

    switch(cam_slv_addr)
    {
    case OV7725_SLV_ADDR: // Same for OV7690.
        cambus_readb(cam_slv_addr, OV_CHIP_ID, &chip_id);
        break;
    case MT9V034_SLV_ADDR:
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_7);
        cambus_readb(cam_slv_addr, ON_CHIP_ID, &chip_id);
        break;
    default:
        break;        
    }

    printf("[chip id][%02X]\r\n", chip_id);

    switch(chip_id)
    {
    case OV7725_ID:
        vio_status.cam_id = OV7725_ID;
        vio_status.gs_bpp = 2;
		printf("[CAM CHIP][OV7725]\r\n");
        ov7725_init();
        break;
    case MT9V034_ID:
        vio_status.cam_id = MT9V034_ID;
        vio_status.gs_bpp = 1;
		printf("[CAM CHIP][MT9V034]\r\n");
        mt9v034_init();
        break;
    default:
        break;        
    }
}

void dcmi_dma_start(void)
{

  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
  __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_LINE);

  HAL_DCMI_Stop(&hdcmi); 
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dcmi_image_buffer, vio_status.cam_frame_size/4*vio_status.gs_bpp);
  __HAL_DCMI_ENABLE(&hdcmi);

  while ((DCMI->CR & DCMI_CR_CAPTURE) != 0)
  {
	  icm20948_transmit();
  }
  LCD_Show_Cam(dcmi_image_buffer,vio_status.cam_frame_size);
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
