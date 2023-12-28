#include <string.h>
#include "mt9v034.h"
#include "main.h"
#include "cambus.h"
#include "openvio.h"

#include "dcmi.h"

extern DCMI_HandleTypeDef hdcmi;

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
//static uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data);
static int set_auto_exposure(int enable, int exposure_us);
static int set_framesize(framesize_t framesize);
//static int set_colorbar(int enable); //是否使能测试条码
static int set_hmirror(int enable); //设置水平对称
static int set_vflip(int enable);   //设置竖直对称

void mt9v034_dcmi_init(void)
{

    hdcmi.Instance = DCMI;
    hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
    hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
    hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
    hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
    hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
    if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
*  MT9V034分辨率、帧率修改
*  ----------------
*  752 * 480   max60fps
*  376 * 240   120fps      112fps
*  188 * 120   max240fps   避免消隐区最大193fps 
*  行和列都可以四分频  当行分频时，输出帧率会倍频
*  垂直翻转、水平翻转。
*/
static void _mt9v_frame(uint16_t height,uint16_t width, uint8_t fps)
{
  uint16_t data = 0;
  uint16_t frameRate = 0;
  if((height*4)<=MAX_IMAGE_HEIGHT )   //判断行是否4分频
  {
    height *= 4;
    data |= MT9V034_READ_MODE_ROW_BIN_4;
    if(fps > 193)
      fps = 193;    //最大帧率 193
    if(fps < 1)
      fps = 1;        //最小帧率  1
    
    if(fps > 132)  //fps 过高，不适合一起算
    {
      frameRate = (uint16_t)(-2.0 * fps + 504);  //估算的，不精确
    }
    else
    {
      frameRate = (uint16_t)(132.0 / fps * 240);
    }
  }
  else if((height*2)<=MAX_IMAGE_HEIGHT )  //判断行是否2分频
  {
    height *= 2;
    data |= MT9V034_READ_MODE_ROW_BIN_2;
    
    if(fps > 112)
      fps = 112;    //最大帧率 112
    if(fps < 1)
      fps = 1;        //最小帧率  1
    if(fps > 66)  //fps 过高，不适合一起算
    {
      frameRate = (uint16_t)(-5.2 * fps + 822);
    }
    else
    {
      frameRate = (uint16_t)(66.0 / fps * 480);
    }
  }
  else 
  {
    if(fps > 60) 
      fps = 60;    //最大帧率 60
    if(fps < 1) 
      fps = 1;        //最小帧率  1
    frameRate = (uint16_t)(60.0 / fps * 480);
  }
  if((width*4)<=MAX_IMAGE_WIDTH )                                             //判断列是否4分频   
  {
    width *= 4;
    data |= MT9V034_READ_MODE_COL_BIN_4;
  }
  else if((width*2)<=MAX_IMAGE_WIDTH )                                        //判断列是否2分频
  {
    width *= 2;
    data |= MT9V034_READ_MODE_COL_BIN_2;
  }
  //         水平翻转                     垂直翻转
  data |= (MT9V034_READ_MODE_ROW_FLIP|MT9V034_READ_MODE_COL_FLIP);         //需要翻转的可以打开注释，或者后面PXP转换时翻转也可以  
  
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_READ_MODE, data);                          //写寄存器，配置行分频
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_WINDOW_WIDTH,  width);                     //读取图像的列数  改变此处也可改变图像输出大小，不过会丢失视角
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_WINDOW_HEIGHT, height);                    //读取图像的行数  改变此处也可改变图像输出大小，不过会丢失视角
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_COL_START, MT9V034_COL_START_MIN);   //列开始
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_ROW_START, MT9V034_ROW_START_MIN);         //行开始
  cambus_writew(MT9V034_SLV_ADDR, MT9V034_TOTAL_SHUTTER_WIDTH,frameRate);                 //0x0B 曝光时间 越长帧率越小
}

void set_triggered_mode(int enable)
{
    uint16_t reg;
    int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL, &reg);
    ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL,
                         (reg & (~MT9V034_CHIP_CONTROL_MODE_MASK)) | ((enable != 0) ? MT9V034_CHIP_CONTROL_SNAP_MODE : MT9V034_CHIP_CONTROL_MASTER_MODE));
}

int mt9v034_init(void)
{
    uint16_t chip_version = 0;
    int retry_num = 1, err = 0;

    ////////////////////////////////////////////////////////////////////////////////

    while (!chip_version && retry_num-- > 0)
        chip_version = mt9v034_ReadReg16(MTV_CHIP_VERSION_REG);

    if (chip_version != 0x1324)
    {
        printf("[mt9v034 chip version][read err]\r\n");
        return -1;
    }
    printf("[mt9v034 chip version][ok]\r\n");

    mt9v034_dcmi_init();

  cambus_writew(MT9V034_SLV_ADDR, 0x01, 0x0001);   //COL_WINDOW_START_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x02, 0x0004);   //ROW_WINDOW_START_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x03, 0x01E0);   //ROW_WINDOW_SIZE_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x04, 0x02F0);   //COL_WINDOW_SIZE_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x05, 0x005E);   //HORZ_BLANK_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x06, 0x002D);   //VERT_BLANK_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x07, 0x0188);   //CONTROL_MODE_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x08, 0x01BB);   //COARSE_SHUTTER_WIDTH_1_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x09, 0x01D9);   //COARSE_SHUTTER_WIDTH_2_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x0A, 0x0164);   //SHUTTER_WIDTH_CONTROL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x0B, 0x01E0);   //COARSE_SHUTTER_WIDTH_TOTAL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x0C, 0x0000);   //RESET_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x0D, 0x0300);   //READ_MODE_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x0E, 0x0000);   //READ_MODE2_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x0F, 0x0100);   //PIXEL_OPERATION_MODE
  cambus_writew(MT9V034_SLV_ADDR, 0x10, 0x0040);   //RAMP_START_DELAY
  cambus_writew(MT9V034_SLV_ADDR, 0x11, 0x8042);   //OFFSET_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x12, 0x0022);   //AMP_RESET_BAR_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x13, 0x2D32);   //5T_PIXEL_RESET_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x14, 0x0E02);   //4T_PIXEL_RESET_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x15, 0x0E32);   //TX_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x16, 0x2802);   //5T_PIXEL_SHS_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x17, 0x3E38);   //4T_PIXEL_SHS_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x18, 0x3E38);   //5T_PIXEL_SHR_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x19, 0x2802);   //4T_PIXEL_SHR_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x1A, 0x0428);   //COMPARATOR_RESET_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x1B, 0x0000);   //LED_OUT_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x1C, 0x0302);   //DATA_COMPRESSION
  cambus_writew(MT9V034_SLV_ADDR, 0x1D, 0x0040);   //ANALOG_TEST_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x1E, 0x0000);   //SRAM_TEST_DATA_ODD
  cambus_writew(MT9V034_SLV_ADDR, 0x1F, 0x0000);   //SRAM_TEST_DATA_EVEN
  cambus_writew(MT9V034_SLV_ADDR, 0x20, 0x03C7);   //BOOST_ROW_EN
  cambus_writew(MT9V034_SLV_ADDR, 0x21, 0x0020);   //I_VLN_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x22, 0x0020);   //I_VLN_AMP_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x23, 0x0010);   //I_VLN_CMP_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x24, 0x001B);   //I_OFFSET_CONTROL
  //    MTV_IICWriteReg1G=0x25, 0x0000); // I_BANDGAP_CONTROL - TRIMMED PER DIE
  cambus_writew(MT9V034_SLV_ADDR, 0x26, 0x0004);   //I_VLN_VREF_ADC_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x27, 0x000C);   //I_VLN_STEP_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x28, 0x0010);   //I_VLN_BUF_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x29, 0x0010);   //I_MASTER_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2A, 0x0020);   //I_VLN_AMP_60MHZ_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2B, 0x0004);   //VREF_AMP_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2C, 0x0000);   //VREF_ADC_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2D, 0x0004);   //VBOOST_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2E, 0x0007);   //V_HI_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x2F, 0x0003);   //V_LO_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x30, 0x0003);   //V_AMP_CAS_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x31, 0x0027);   //V1_CONTROL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x32, 0x001A);   //V2_CONTROL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x33, 0x0005);   //V3_CONTROL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x34, 0x0003);   //V4_CONTROL_CONTEXTA
  cambus_writew(MT9V034_SLV_ADDR, 0x35, 0x0010);   //GLOBAL_GAIN_CONTEXTA_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x36, 0x8010);   //GLOBAL_GAIN_CONTEXTB_REG
  cambus_writew(MT9V034_SLV_ADDR, 0x37, 0x0000);   //VOLTAGE_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x38, 0x0000);   //IDAC_VOLTAGE_MONITOR
  cambus_writew(MT9V034_SLV_ADDR, 0x39, 0x0027);   //V1_CONTROL_CONTEXTB
  cambus_writew(MT9V034_SLV_ADDR, 0x3A, 0x0026);   //V2_CONTROL_CONTEXTB
  cambus_writew(MT9V034_SLV_ADDR, 0x3B, 0x0005);   //V3_CONTROL_CONTEXTB
  cambus_writew(MT9V034_SLV_ADDR, 0x3C, 0x0003);   //V4_CONTROL_CONTEXTB
  cambus_writew(MT9V034_SLV_ADDR, 0x40, 0x0080);   //DARK_AVG_THRESHOLDS
  cambus_writew(MT9V034_SLV_ADDR, 0x46, 0x231D);   //CALIB_CONTROL_REG (AUTO)
  cambus_writew(MT9V034_SLV_ADDR, 0x47, 0x0080);   //STEP_SIZE_AVG_MODE
  cambus_writew(MT9V034_SLV_ADDR, 0x48, 0x0020);   //ROW_NOISE_CONTROL
  cambus_writew(MT9V034_SLV_ADDR, 0x4C, 0x0002);   //NOISE_CONSTANT
  cambus_writew(MT9V034_SLV_ADDR, 0x60, 0x0000);   //PIXCLK_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0x67, 0x0000);   //TEST_DATA
  // cambus_writew(MT9V034_SLV_ADDR, 0x6C, 0x0000);   //TILE_X0_Y0
  // cambus_writew(MT9V034_SLV_ADDR, 0x70, 0x0000);   //TILE_X1_Y0
  // cambus_writew(MT9V034_SLV_ADDR, 0x71, 0x002A);   //TILE_X2_Y0
  // cambus_writew(MT9V034_SLV_ADDR, 0x72, 0x0000);   //TILE_X3_Y0
  // cambus_writew(MT9V034_SLV_ADDR, 0x7F, 0x0000);   //TILE_X4_Y0
  // cambus_writew(MT9V034_SLV_ADDR, 0x99, 0x0000);   //TILE_X0_Y1
  // cambus_writew(MT9V034_SLV_ADDR, 0x9A, 0x0096);   //TILE_X1_Y1
  // cambus_writew(MT9V034_SLV_ADDR, 0x9B, 0x012C);   //TILE_X2_Y1
  // cambus_writew(MT9V034_SLV_ADDR, 0x9C, 0x01C2);   //TILE_X3_Y1
  // cambus_writew(MT9V034_SLV_ADDR, 0x9D, 0x0258);   //TILE_X4_Y1
  // cambus_writew(MT9V034_SLV_ADDR, 0x9E, 0x02F0);   //TILE_X0_Y2
  // cambus_writew(MT9V034_SLV_ADDR, 0x9F, 0x0000);   //TILE_X1_Y2
  // cambus_writew(MT9V034_SLV_ADDR, 0xA0, 0x0060);   //TILE_X2_Y2
  // cambus_writew(MT9V034_SLV_ADDR, 0xA1, 0x00C0);   //TILE_X3_Y2
  // cambus_writew(MT9V034_SLV_ADDR, 0xA2, 0x0120);   //TILE_X4_Y2
  // cambus_writew(MT9V034_SLV_ADDR, 0xA3, 0x0180);   //TILE_X0_Y3
  // cambus_writew(MT9V034_SLV_ADDR, 0xA4, 0x01E0);   //TILE_X1_Y3
  // cambus_writew(MT9V034_SLV_ADDR, 0xA5, 0x003A);   //TILE_X2_Y3
  // cambus_writew(MT9V034_SLV_ADDR, 0xA6, 0x0002);   //TILE_X3_Y3
  // cambus_writew(MT9V034_SLV_ADDR, 0xA8, 0x0000);   //TILE_X4_Y3
  // cambus_writew(MT9V034_SLV_ADDR, 0xA9, 0x0002);   //TILE_X0_Y4
  // cambus_writew(MT9V034_SLV_ADDR, 0xAA, 0x0002);   //TILE_X1_Y4
  // cambus_writew(MT9V034_SLV_ADDR, 0xAB, 0x0040);   //TILE_X2_Y4
  // cambus_writew(MT9V034_SLV_ADDR, 0xAC, 0x0001);   //TILE_X3_Y4
  // cambus_writew(MT9V034_SLV_ADDR, 0xAD, 0x01E0);   //TILE_X4_Y4
  // cambus_writew(MT9V034_SLV_ADDR, 0xAE, 0x0014);   //X0_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xAF, 0x0000);   //X1_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB0, 0xABE0);   //X2_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB1, 0x0002);   //X3_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB2, 0x0010);   //X4_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB3, 0x0010);   //X5_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB4, 0x0000);   //Y0_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB5, 0x0000);   //Y1_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB6, 0x0000);   //Y2_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xB7, 0x0000);   //Y3_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xBF, 0x0016);   //Y4_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xC0, 0x000A);   //Y5_SLASH5
  // cambus_writew(MT9V034_SLV_ADDR, 0xC2, 0x18D0);   //DESIRED_BIN
  // cambus_writew(MT9V034_SLV_ADDR, 0xC3, 0x007F);   //EXP_SKIP_FRM_H
  // cambus_writew(MT9V034_SLV_ADDR, 0xC4, 0x007F);   //EXP_LPF
  // cambus_writew(MT9V034_SLV_ADDR, 0xC5, 0x007F);   //GAIN_SKIP_FRM
  // cambus_writew(MT9V034_SLV_ADDR, 0xC6, 0x0000);   //GAIN_LPF_H
  // cambus_writew(MT9V034_SLV_ADDR, 0xC7, 0x4416);   //MAX_GAIN
  // cambus_writew(MT9V034_SLV_ADDR, 0xC8, 0x4421);   //MIN_COARSE_EXPOSURE
  // cambus_writew(MT9V034_SLV_ADDR, 0xC9, 0x0001);   //MAX_COARSE_EXPOSURE
  // cambus_writew(MT9V034_SLV_ADDR, 0xCA, 0x0004);   //BIN_DIFF_THRESHOLD
  // cambus_writew(MT9V034_SLV_ADDR, 0xCB, 0x01E0);   //AUTO_BLOCK_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xCC, 0x02F0);   //PIXEL_COUNT
  // cambus_writew(MT9V034_SLV_ADDR, 0xCD, 0x005E);   //LVDS_MASTER_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xCE, 0x002D);   //LVDS_SHFT_CLK_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xCF, 0x01DE);   //LVDS_DATA_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xD0, 0x01DF);   //LVDS_DATA_STREAM_LATENCY
  // cambus_writew(MT9V034_SLV_ADDR, 0xD1, 0x0164);   //LVDS_INTERNAL_SYNC
  // cambus_writew(MT9V034_SLV_ADDR, 0xD2, 0x0001);   //LVDS_USE_10BIT_PIXELS
  // cambus_writew(MT9V034_SLV_ADDR, 0xD3, 0x0000);   //STEREO_ERROR_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xD4, 0x0000);   //INTERLACE_FIELD_VBLANK
  // cambus_writew(MT9V034_SLV_ADDR, 0xD5, 0x0104);   //IMAGE_CAPTURE_NUM
  // cambus_writew(MT9V034_SLV_ADDR, 0xD6, 0x0000);   //ANALOG_CONTROLS
  // cambus_writew(MT9V034_SLV_ADDR, 0xD7, 0x0000);   //AB_PULSE_WIDTH_REG
  // cambus_writew(MT9V034_SLV_ADDR, 0xD8, 0x0000);   //TX_PULLUP_PULSE_WIDTH_REG
  // cambus_writew(MT9V034_SLV_ADDR, 0xD9, 0x0000);   //RST_PULLUP_PULSE_WIDTH_REG
  // cambus_writew(MT9V034_SLV_ADDR, 0xF0, 0x0000);   //NTSC_FV_CONTROL
  // cambus_writew(MT9V034_SLV_ADDR, 0xFE, 0xBEEF);   //NTSC_HBLANK
  
  _mt9v_frame(480,752,60);

  // cambus_writew(MT9V034_SLV_ADDR, 0x2C, 0x0004);                                            //参考电压设置   1.4v 
  // // cambus_writew(MT9V034_SLV_ADDR, MT9V034_ANALOG_CTRL, MT9V034_ANTI_ECLIPSE_ENABLE);        //反向腐蚀
  // cambus_writew(MT9V034_SLV_ADDR, MTV_MAX_GAIN_REG, 10);                                //0xAB  最大模拟增益     64
  // cambus_writew(MT9V034_SLV_ADDR, MTV_AGC_AEC_PIXEL_COUNT_REG, 0xB0);                   //0xB0  用于AEC/AGC直方图像素数目,最大44000   IMAGEH*IMAGEW  
  // cambus_writew(MT9V034_SLV_ADDR, MTV_ADC_RES_CTRL_REG, 0x0303);                        //0x1C  here is the way to regulate darkness :)    
  // cambus_writew(MT9V034_SLV_ADDR, 0x13,0x2D2E);                                             //We also recommended using R0x13 = 0x2D2E with this setting for better column FPN.  
  // cambus_writew(MT9V034_SLV_ADDR, 0x20,0x03c7);                                             //0x01C7对比度差，发白；0x03C7对比度提高 Recommended by design to improve performance in HDR mode and when frame rate is low.
  // cambus_writew(MT9V034_SLV_ADDR, 0x24,0x0010);                                             //Corrects pixel negative dark offset when global reset in R0x20[9] is enabled.
  // cambus_writew(MT9V034_SLV_ADDR, 0x2B,0x0003);                                             //Improves column FPN.
  // cambus_writew(MT9V034_SLV_ADDR, 0x2F,0x0003);                                             //Improves FPN at near-saturation.  
  // cambus_writew(MT9V034_SLV_ADDR, MT9V034_SHUTTER_WIDTH_CONTROL,0x0164);                    //0x0A Coarse Shutter IMAGEW Control 
  // cambus_writew(MT9V034_SLV_ADDR, MTV_V2_CTRL_REG_A, 0x001A);                           //0x32   0x001A  
  // cambus_writew(MT9V034_SLV_ADDR, MTV_HDR_ENABLE_REG,0x0103);                           //0x0F High Dynamic Range enable,bit is set (R0x0F[1]=1), the sensor uses black level correction values from one green plane, which are applied to all colors.
  // cambus_writew(MT9V034_SLV_ADDR, MTV_AGC_AEC_DESIRED_BIN_REG, 40);                     //0xA5  图像亮度  60  1-64
  // cambus_writew(MT9V034_SLV_ADDR, MT9V034_ANALOG_GAIN,0x8010);
  
  
  //set_triggered_mode(0);
  //cambus_writew(MT9V034_SLV_ADDR, MT9V034_PIXEL_CLOCK, MT9V034_PIXEL_CLOCK_INV_PXL_CLK);
    mt9v034_config(vio_status.cam_frame_size_num);

    // set_colorbar(0);
    set_vflip(0);
    set_hmirror(0);
    set_auto_exposure(0, 200); //曝光设置
    
	//cambus_writew(MT9V034_SLV_ADDR, MT9V034_RESET, 0x03);                                     //0x0c  复位 

    // uint16_t chip_control;
    // int enable = 0;
    // int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL, &chip_control);
    // ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_CHIP_CONTROL,
    // 					 (chip_control & (~MT9V034_CHIP_CONTROL_MODE_MASK)) | ((enable != 0) ? MT9V034_CHIP_CONTROL_SNAP_MODE : MT9V034_CHIP_CONTROL_MASTER_MODE));

    return err;
}

void mt9v034_config(framesize_t frame_size_num)
{
    vio_status.cam_frame_size_num = frame_size_num;
    vio_status.cam_frame_size = resolution[frame_size_num][0] * resolution[frame_size_num][1];
    set_framesize(frame_size_num);
}

void mt9v034_exposure(int exposure)
{
    //printf("exposure %d\r\n",exposure);
    if (exposure < 0)
    {
        set_auto_exposure(1, -exposure);
    }
    else
    {
        set_auto_exposure(0, exposure);
    }
}

int IM_MIN(int a, int b)
{
    if (a > b)
        return b;
    else
        return a;
}

#define MICROSECOND_CLKS (1000000)
#define MT9V034_XCLK_FREQ 24000000



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

        ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_TOTAL_SHUTTER_WIDTH, fine_time);
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

//static int set_colorbar(int enable) //是否使能测试条码
//{
//	uint16_t test;
//	int ret = cambus_readw(MT9V034_SLV_ADDR, MT9V034_TEST_PATTERN, &test);
//	ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_TEST_PATTERN,
//						 (test & (~(MT9V034_TEST_PATTERN_ENABLE | MT9V034_TEST_PATTERN_GRAY_MASK))) | ((enable != 0) ? (MT9V034_TEST_PATTERN_ENABLE | MT9V034_TEST_PATTERN_GRAY_VERTICAL) : 0));

//	return ret;
//}

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
    // ret |= cambus_writew(MT9V034_SLV_ADDR, MT9V034_HORIZONTAL_BLANKING,
    //                      MT9V034_HORIZONTAL_BLANKING_DEF + (MT9V034_MAX_WIDTH - IM_MIN(width * read_mode_mul, 640)));

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
#if I2C_GPIO_ENABLE
    i2c_master_read_mem(mt9v034_DEVICE_WRITE_ADDRESS, address, tmp, 2);
#else
    HAL_I2C_Mem_Read(&hi2c1, mt9v034_DEVICE_WRITE_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, tmp, 2, 1000);
#endif
    return tmp[0] << 8 | tmp[1];
}

///**
//  * @brief  Writes to a specific Camera register
//  */
//static uint8_t mt9v034_WriteReg16(uint16_t address, uint16_t Data)
//{
//	uint8_t tmp[4];
//	int8_t ret;
//	/*
//	tmp[0] = address;
//	tmp[1] = Data >> 8;
//	tmp[2] = Data;
//*/
//	tmp[0] = address;
//	tmp[1] = (uint8_t)(Data >> 8);
//	tmp[2] = 0xf0;
//	tmp[3] = (uint8_t)Data;
//#if I2C_GPIO_ENABLE
//#if 0
//	i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, tmp, 3);
//#else
//	ret = i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, &tmp[0], 2);
//	ret = i2c_master_transmit(mt9v034_DEVICE_WRITE_ADDRESS, &tmp[2], 2);
//#endif
//#else
//	ret = HAL_I2C_Master_Transmit(&hi2c1, mt9v034_DEVICE_WRITE_ADDRESS, tmp, 3, 100);
//#endif
//	return ret;
//}
