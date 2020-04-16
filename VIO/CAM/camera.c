#include "camera.h"

#include "openvio.h"
#include "mt9v034.h"
#include "ov7725.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

DMA_BUFFER uint8_t dcmi_image_buffer[CAM_PACKAGE_MAX_SIZE * 2] = {0};

int frame_count = 0;
int line_cnt = 0, count = 0, start = 0;

HAL_StatusTypeDef USER_DCMI_Start_DMA(DCMI_HandleTypeDef *hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length);

SemaphoreHandle_t xSemaphore;
void camera_init(void)
{
    uint8_t chip_id, cam_slv_addr;

	xSemaphore = xSemaphoreCreateBinary();
	
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    cam_slv_addr = cambus_scan();

    printf("[cam slv addr][%02X]\r\n", cam_slv_addr);

    switch (cam_slv_addr)
    {
    case OV7725_SLV_ADDR: // Same for OV7690.
		HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_8); 
        cambus_readb(cam_slv_addr, OV_CHIP_ID, &chip_id);
        break;
    case MT9V034_SLV_ADDR:
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_3); //3 32MHZ,4 24MHZ
        cambus_readb(cam_slv_addr, ON_CHIP_ID, &chip_id);
        break;
    default:
        break;
    }

    printf("[chip id][%02X]\r\n", chip_id);

    switch (chip_id)
    {
    case OV7725_ID:
        vio_status.cam_id = OV7725_ID;
        vio_status.gs_bpp = 2;
        printf("[CAM CHIP][OV7725]\r\n");
        ov7725_init();

        break;
    case MT9V034_ID:
        vio_status.pixformat = PIXFORMAT_GRAYSCALE;
        vio_status.cam_id = MT9V034_ID;
        vio_status.gs_bpp = 1;
        printf("[CAM CHIP][MT9V034]\r\n");
        mt9v034_init();
        break;
    default:
        break;
    }
}

uint8_t str[20],cnt=0;

extern QueueHandle_t xQueue;
void USER_DCMI_MemDMAXferCplt(uint32_t data, uint32_t size)
{
	if(vio_status.cam_frame_size * vio_status.gs_bpp > CAM_PACKAGE_MAX_SIZE)
	{
		struct USB_FRAME_STRUCT usb_frame_s;
		usb_frame_s.addr = (uint8_t *)data;
		usb_frame_s.len = size;
		xQueueSendFromISR( xQueue, ( void * )&usb_frame_s, ( TickType_t ) 0 );
	}
    //while(openvio_usb_send(SENSOR_USB_CAM,(uint8_t *)data, size)!=0);
}

void dcmi_dma_start(void)
{
    for(int i=0;i<20;i++)
        str[i]='\0';

    cnt=0;

    //__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
    //__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_LINE);

    //while (CDC_Transmit_HS("CAM", 3) != 0);

	while(get_usb_tx_state() != 0)
	{
		//osDelay(1);
	}
	
	struct USB_FRAME_STRUCT usb_frame_s;
	usb_frame_s.addr = (uint8_t *)"CAMERA";
	usb_frame_s.len = 6;	
	xQueueSend( xQueue, ( void * )&usb_frame_s, ( TickType_t ) 0 );
    
	__HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME);
    USER_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dcmi_image_buffer, vio_status.cam_frame_size / 4 * vio_status.gs_bpp);
    //__HAL_DCMI_ENABLE(&hdcmi);

//    while ((DCMI->CR & DCMI_CR_CAPTURE) != 0)
//    {
//        //icm20948_transmit();
//		//osDelay(100);
//    }
	
    xSemaphoreTake( xSemaphore, portMAX_DELAY );

	HAL_DCMI_Stop(&hdcmi);

	if(vio_status.cam_frame_size * vio_status.gs_bpp <= CAM_PACKAGE_MAX_SIZE)
	{
		usb_frame_s.addr = (uint8_t *)dcmi_image_buffer;
		usb_frame_s.len = vio_status.cam_frame_size * vio_status.gs_bpp;
		xQueueSend( xQueue, ( void * )&usb_frame_s, ( TickType_t ) 0 );
	}
	
    
//	osDelay(4000);
//	printf("%s\r\n",str);
//	osDelay(4000);
	//printf("%d\r\n",count_value);
    //LCD_Show_Cam(dcmi_image_buffer,vio_status.cam_frame_size);
}

//void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
//{

//}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	static BaseType_t xHigherPriorityTaskWoken;
	
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}

//void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
//{

//}

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

static void USER_DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
	
  uint32_t tmp;

  DCMI_HandleTypeDef* hdcmi = ( DCMI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
    str[cnt++] = '|';
  if(hdcmi->XferCount != 0U)
  {
    /* Update memory 0 address location */
    tmp = ((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR) & DMA_SxCR_CT);
    if(((hdcmi->XferCount % 2U) == 0U) && (tmp != 0U))
    {
        str[cnt++] = '1';
      tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR;
      //(void) HAL_DMAEx_ChangeMemory(hdcmi->DMA_Handle, (tmp + (8U*hdcmi->XferSize)), MEMORY0);
      USER_DCMI_MemDMAXferCplt(tmp,hdcmi->XferSize*4);
      hdcmi->XferCount--;
    }
    /* Update memory 1 address location */
    else if((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) == 0U)
    {
        str[cnt++] = '2';
      tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR;
      //(void) HAL_DMAEx_ChangeMemory(hdcmi->DMA_Handle, (tmp + (8U*hdcmi->XferSize)), MEMORY1);
      USER_DCMI_MemDMAXferCplt(tmp,hdcmi->XferSize*4);
      hdcmi->XferCount--;
    }
    else
    {
        str[cnt++] = '3';
      /* Nothing to do */
    }
  }
  /* Update memory 0 address location */
  else if((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) != 0U)
  {
      str[cnt++] = '4';
      tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR;
      USER_DCMI_MemDMAXferCplt((uint32_t)tmp,hdcmi->XferSize*4);
    //((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR = hdcmi->pBuffPtr;
  }
  /* Update memory 1 address location */
  else if((((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->CR & DMA_SxCR_CT) == 0U)
  {
      str[cnt++] = '5';
      tmp = ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR;
		  
	  USER_DCMI_MemDMAXferCplt((uint32_t)tmp,hdcmi->XferSize*4);
    //tmp = hdcmi->pBuffPtr;
    //((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR = (tmp + (4U*hdcmi->XferSize));
    hdcmi->XferCount = hdcmi->XferTransferNumber;
  }
  else
  {
    /* Nothing to do */
  }

  /* Check if the frame is transferred */
  if(hdcmi->XferCount == hdcmi->XferTransferNumber)
  {
    //   if(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR > ((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR)
    //   {
    //       str[cnt++] = '6';
    //       USER_DCMI_MemDMAXferCplt(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M1AR,hdcmi->XferSize*4);
    //   }
    //   else
    //   {
    //       str[cnt++] = '7';
    //       USER_DCMI_MemDMAXferCplt(((DMA_Stream_TypeDef *)(hdcmi->DMA_Handle->Instance))->M0AR,hdcmi->XferSize*4);
    //   }
        
    str[cnt++] = '8';
    /* Enable the Frame interrupt */
    __HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);

    /* When snapshot mode, set dcmi state to ready */
    if((hdcmi->Instance->CR & DCMI_CR_CM) == DCMI_MODE_SNAPSHOT)
    {
        str[cnt++] = '9';
      hdcmi->State= HAL_DCMI_STATE_READY;
		
    }
  }
}

HAL_StatusTypeDef USER_DCMI_Start_DMA(DCMI_HandleTypeDef* hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length)
{
  /* Initialize the second memory address */
  uint32_t SecondMemAddress;

  /* Check function parameters */
  assert_param(IS_DCMI_CAPTURE_MODE(DCMI_Mode));

  /* Process Locked */
  __HAL_LOCK(hdcmi);

  /* Lock the DCMI peripheral state */
  hdcmi->State = HAL_DCMI_STATE_BUSY;

  /* Enable DCMI by setting DCMIEN bit */
  __HAL_DCMI_ENABLE(hdcmi);

  /* Configure the DCMI Mode */
  hdcmi->Instance->CR &= ~(DCMI_CR_CM);
  hdcmi->Instance->CR |=  (uint32_t)(DCMI_Mode);

  /* Set the DMA memory0 conversion complete callback */
  hdcmi->DMA_Handle->XferCpltCallback = USER_DCMI_DMAXferCplt;

  /* Set the DMA error callback */
  hdcmi->DMA_Handle->XferErrorCallback = NULL;

  /* Set the dma abort callback */
  hdcmi->DMA_Handle->XferAbortCallback = NULL;

  /* Reset transfer counters value */
  hdcmi->XferCount = 0;
  hdcmi->XferTransferNumber = 0;
  if(Length <= CAM_PACKAGE_MAX_SIZE/4)
  {
    /* Enable the DMA Stream */
    if (HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, Length) != HAL_OK)
    {
      /* Set Error Code */
      hdcmi->ErrorCode = HAL_DCMI_ERROR_DMA;
      /* Change DCMI state */
      hdcmi->State = HAL_DCMI_STATE_READY;
      /* Release Lock */
      __HAL_UNLOCK(hdcmi);
      /* Return function status */
      return HAL_ERROR;
    }
  }
  else /* DCMI_DOUBLE_BUFFER Mode */
  {
    /* Set the DMA memory1 conversion complete callback */
    hdcmi->DMA_Handle->XferM1CpltCallback = USER_DCMI_DMAXferCplt;

    /* Initialize transfer parameters */
    hdcmi->XferCount = 1;
    hdcmi->XferSize = Length;
    hdcmi->pBuffPtr = pData;

    /* Get the number of buffer */
    while(hdcmi->XferSize > CAM_PACKAGE_MAX_SIZE/4)
    {
      hdcmi->XferSize = (hdcmi->XferSize/2U);
      hdcmi->XferCount = hdcmi->XferCount*2U;
    }
    /* Update DCMI counter  and transfer number*/
    hdcmi->XferCount = (hdcmi->XferCount - 2U);
    hdcmi->XferTransferNumber = hdcmi->XferCount;
    /* Update second memory address */
    SecondMemAddress = (uint32_t)(pData + (4U*hdcmi->XferSize));

    /* Start DMA multi buffer transfer */
    if (HAL_DMAEx_MultiBufferStart_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, SecondMemAddress, hdcmi->XferSize) != HAL_OK)
    {
      /* Set Error Code */
      hdcmi->ErrorCode = HAL_DCMI_ERROR_DMA;
      /* Change DCMI state */
      hdcmi->State = HAL_DCMI_STATE_READY;
      /* Release Lock */
      __HAL_UNLOCK(hdcmi);
      /* Return function status */
      return HAL_ERROR;
    }
  }

  /* Enable Capture */
  hdcmi->Instance->CR |= DCMI_CR_CAPTURE;

  /* Release Lock */
  __HAL_UNLOCK(hdcmi);

  /* Return function status */
  return HAL_OK;
}