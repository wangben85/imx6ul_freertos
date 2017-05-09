/*
* Copyright (c) 2017, NXP Semiconductors, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "fsl_elcdif.h"
#include "fsl_pxp.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"

#include "board.h"
#include "fsl_debug_console.h"

#include "fsl_gpio.h"
#include "fsl_csi.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_ov5640.h"
#include "pin_mux.h"

//For QRdecoder
#include "zxing_bridge.h"
#include <string.h>
/*******************************************************************************
* Definitions
******************************************************************************/

/* LCD definition. */
#define APP_ELCDIF LCDIF

#define APP_LCD_HEIGHT 272
#define APP_LCD_WIDTH 480
//test
//#define APP_LCD_HEIGHT 240
//#define APP_LCD_WIDTH 320

#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_LCD_POL_FLAGS \
(kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

/* Display. */
#define LCD_DISP_GPIO GPIO5
#define LCD_DISP_GPIO_PIN 9
/* Back light. */
#define LCD_BL_GPIO GPIO1
#define LCD_BL_GPIO_PIN 8

/* Camera definition. */
//#define APP_CAMERA_HEIGHT 240
//#define APP_CAMERA_WIDTH 320
#define APP_CAMERA_HEIGHT 272
#define APP_CAMERA_WIDTH 480

#define APP_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)

#define OV5640_I2C I2C2

#define APP_PXP PXP

/* Frame buffer data alignment. */
#define FRAME_BUFFER_ALIGN 64


/* Pixel format YUV422, bytesPerPixel is 2. */
#define APP_BPP 2

#define APP_CAMERA_FRAME_BUFFER_COUNT 4

/*******************************************************************************
* Prototypes
******************************************************************************/
extern camera_device_handle_t cameraDevice;
extern camera_receiver_handle_t cameraReceiver;

/*******************************************************************************
* Variables
******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(
                              static uint16_t s_cameraBuffer[APP_CAMERA_FRAME_BUFFER_COUNT][APP_CAMERA_HEIGHT][APP_CAMERA_WIDTH],
                              FRAME_BUFFER_ALIGN);

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t decoderBuffer[2][APP_LCD_HEIGHT][APP_LCD_WIDTH], FRAME_BUFFER_ALIGN);
//AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t decoderBuffer[2][APP_CAMERA_HEIGHT][APP_CAMERA_WIDTH], FRAME_BUFFER_ALIGN);
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_lcdBuffer[2][APP_LCD_HEIGHT][APP_LCD_WIDTH], FRAME_BUFFER_ALIGN);

//QRdecoder
uint8_t QR_BUFFER[APP_CAMERA_WIDTH*8];// 8 lines
static char format[32];
static char result[256];
volatile int ret;
volatile uint8_t *test_results ;

uint32_t p;  //24bit color value(RGB)
uint8_t color_y; //8bit color value(monochrome)

/*******************************************************************************
* Code
******************************************************************************/

extern void CSI_DriverIRQHandler(void);

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
  volatile uint32_t i = 0x100U;
  
  gpio_pin_config_t config = {
    kGPIO_DigitalOutput, 0,
  };
  
  /* LCD power on. */
  BOARD_NXP74LV595_SetValue(kNXP74LV595_LCD_nPWREN, kSignal_NXP74LV595_Low);
  
  /* Reset the LCD. */
  GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);
  
  GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);
  
  while (i--)
  {
  }
  
  GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);//set LCD_DISP=1, enable LCD
  
  /* Backlight. */
  config.outputLogic = 1;
  GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

void BOARD_InitLcdifPixClock(void)
{
  /*
  *
  HFP: Horizon front porch
  HBP: Horizon back porch
  VFP: Vertical front porch
  VBP: Vertical back porch
  HDP:Horizon display period = HSW
  VDP:  Vertical display period =VSW
  HTP = HSYNC(480) + HDP + HFP + HBP
  VTR = VSYNC(272) + VDP + VFP + VBP
  f dot_clk = pixel clock
  f dot_clk = f v * VTR * HTP
  f v = vertical frequency  (60HZ)
  *
  * The desired output frame rate is 60Hz. So the pixel clock frequency is:
  * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
  * Here set the LCDIF pixel clock to 9.3M.
  */
  
  /*
  * Initialize the Video PLL.
  * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
  */
  clock_video_pll_config_t config = {
    .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
  };
  
  CLOCK_InitVideoPll(&config);
  
  /*
  * 000 derive clock from PLL2
  * 001 derive clock from PLL3 PFD3
  * 010 derive clock from PLL5
  * 011 derive clock from PLL2 PFD0
  * 100 derive clock from PLL2 PFD1
  * 101 derive clock from PLL3 PFD1
  */
  CLOCK_SetMux(kCLOCK_Lcdif1PreMux, 2);//Selector for lcdif1 root clock pre-multiplexer

  // 93 Mhz /5 /2 = 9.3Mhz
  CLOCK_SetDiv(kCLOCK_Lcdif1PreDiv, 4);//Pre-divider for lcdif1 clock
  
  CLOCK_SetDiv(kCLOCK_Lcdif1Div, 1);//Post-divider for lcdif1 clock.
  
  /*
  * 000 derive clock from divided pre-muxed lcdif1 clock
  * 001 derive clock from ipp_di0_clk
  * 010 derive clock from ipp_di1_clk
  * 011 derive clock from ldb_di0_clk
  * 100 derive clock from ldb_di1_clk
  */
  CLOCK_SetMux(kCLOCK_Lcdif1Mux, 0);
}

/* OV5640 connect to CSI. */
static csi_resource_t csiResource = {
  .csiBase = CSI,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t cameraReceiver = {
  .resource = &csiResource, .ops = &csi_ops, .privateData = &csiPrivateData,
};

static void BOARD_PullCameraResetPin(bool pullUp)
{
  if (pullUp)
  {
    BOARD_NXP74LV595_SetValue(kNXP74LV595_CSI_RST, kSignal_NXP74LV595_High);
  }
  else
  {
    BOARD_NXP74LV595_SetValue(kNXP74LV595_CSI_RST, kSignal_NXP74LV595_Low);
  }
}

static void BOARD_PullCameraPowerDownPin(bool pullUp)
{
  if (pullUp)
  {
    BOARD_NXP74LV595_SetValue(kNXP74LV595_CSI_PWDN, kSignal_NXP74LV595_High);
  }
  else
  {
    BOARD_NXP74LV595_SetValue(kNXP74LV595_CSI_PWDN, kSignal_NXP74LV595_Low);
  }
}

static ov5640_resource_t ov5640Resource = {
  .sccbI2C = OV5640_I2C, .pullResetPin = BOARD_PullCameraResetPin, .pullPowerDownPin = BOARD_PullCameraPowerDownPin,
};

camera_device_handle_t cameraDevice = {
  .resource = &ov5640Resource, .ops = &ov5640_ops,
};

void BOARD_InitCameraResource(void)
{
  i2c_master_config_t masterConfig;
  uint32_t sourceClock;
  
  /*
  * masterConfig->baudRate_Bps = 100000U;
  * masterConfig->enableHighDrive = false;
  * masterConfig->enableStopHold = false;
  * masterConfig->glitchFilterWidth = 0U;
  * masterConfig->enableMaster = true;
  */
  I2C_MasterGetDefaultConfig(&masterConfig);
  masterConfig.baudRate_Bps = 100000;
  
  sourceClock = (CLOCK_GetFreq(kCLOCK_IpgClk) / (CLOCK_GetDiv(kCLOCK_PerclkDiv) + 1U));
  
  I2C_MasterInit(OV5640_I2C, &masterConfig, sourceClock);
  
  /* CSI MCLK select 24M. */
  CLOCK_SetMux(kCLOCK_CsiMux, 0);
  CLOCK_SetDiv(kCLOCK_CsiDiv, 0);
  CLOCK_EnableClock(kCLOCK_CsiMclk);
}

/*!
* @brief Main function
*/
int main(void)
{
  uint32_t fullCameraBufferAddr;
  
  uint8_t DecoderInactiveFbIdx = 1;
  
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitLcdifPixClock();
  BOARD_InitMemory();
  BOARD_InitDebugConsole();
  BOARD_InitLcd();//set LCD_DISP=1, and back light to enable LCD
  BOARD_InitCameraResource();//enable I2C and CSI MCLK 
  
  /* For PXP LCDIF handshake. */
  IOMUXC_GPR->GPR5 |= 0x00002000;
  
  /* Install IRQ Handler */
  SystemInitIrqTable();
  /* Tricky here: As IRQ handler in example doesn't use parameters, just ignore them */
  SystemInstallIrqHandler(CSI_IRQn, (system_irq_handler_t)(uint32_t)CSI_DriverIRQHandler, NULL);
  
  memset(s_lcdBuffer, 0, sizeof(s_lcdBuffer));
  memset(decoderBuffer, 0, sizeof(decoderBuffer));  
  
  PRINTF("QR decoder demo start...\r\n");
  
  /*
  * Configure the LCDIF.
  */
  const elcdif_rgb_mode_config_t lcdConfig = {
    .panelWidth = APP_LCD_WIDTH, //480
    .panelHeight = APP_LCD_HEIGHT, //272
    .hsw = APP_HSW,
    .hfp = APP_HFP,
    .hbp = APP_HBP,
    .vsw = APP_VSW,
    .vfp = APP_VFP,
    .vbp = APP_VBP,
    .bufferAddr = (uint32_t)s_lcdBuffer[0],// lcd buffer address
    .polarityFlags = APP_LCD_POL_FLAGS,
    .pixelFormat = kELCDIF_PixelFormatRGB565,  /*!< RGB565, two pixel use 32 bits. */
    //.pixelFormat = kELCDIF_PixelFormatRGB888,  //RGB888 unpacked, one pixel uses 32 bits, high byte unused.
    .dataBus = kELCDIF_DataBus24Bit, //24-bit data bus, support RGB888
  };
  
  ELCDIF_RgbModeInit(APP_ELCDIF, &lcdConfig); // LCD RGB mode init  
  
  /* init QR decoder lib */
  cppInit(APP_CAMERA_WIDTH, APP_CAMERA_HEIGHT);		
  
  const camera_config_t cameraConfig = {
    //.pixelFormat = kVIDEO_PixelFormatYUYV, /*!< YUV422, Y-U-Y-V. */
    .pixelFormat = kVIDEO_PixelFormatRGB565, /*!< RGB565. */	
    .bytesPerPixel = APP_BPP, // Pixel format YUV422, bytesPerPixel is 2
    .resolution = FSL_VIDEO_RESOLUTION(APP_CAMERA_WIDTH, APP_CAMERA_HEIGHT), //camera 320 x 240
    .frameBufferLinePitch_Bytes = APP_CAMERA_WIDTH * APP_BPP,  // 320 x 2
    //.interface = kCAMERA_InterfaceCCIR656,  /*!< CCIR656 interface. */
    .interface = kCAMERA_InterfaceGatedClock,    /*!< HSYNC/HREF, VSYNC, and PIXCLK signals are used. */
    .controlFlags = APP_CAMERA_CONTROL_FLAGS,
    .framePerSec = 30,
  };
  
  /*
  * Configure the PXP for color space conversion.
  */
  PXP_Init(APP_PXP);
  
  pxp_ps_buffer_config_t psBufferConfig = {
    //.pixelFormat = kPXP_PsPixelFormatUYVY1P422,/*!< 16-bit pixels (1-plane U0,Y0,V0,Y1 interleaved bytes) */
    .pixelFormat = kPXP_PsPixelFormatRGB565,/*!< 16-bit pixels without alpha. */
    .swapByte = false,
    .bufferAddrU = 0U,
    .bufferAddrV = 0U,
    .pitchBytes = APP_CAMERA_WIDTH * APP_BPP,
  };
  
  PXP_SetProcessSurfaceBackGroundColor(APP_PXP, 0U);
  
  PXP_SetProcessSurfacePosition(APP_PXP, 0U, 0U, APP_CAMERA_WIDTH - 1U, APP_CAMERA_HEIGHT - 1U);
  
  /* Disable AS. */
  PXP_SetAlphaSurfacePosition(APP_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
  
  /* Output config. */
  pxp_output_buffer_config_t outputBufferConfig = {
    .pixelFormat = kPXP_OutputPixelFormatY8, //PXP input output 'Y'  only
    .interlacedMode = kPXP_OutputProgressive,
    .buffer1Addr = 0U,
    .pitchBytes = APP_LCD_WIDTH *1, // 1 bytes = 8bits 
    .width = APP_LCD_WIDTH,
    .height = APP_LCD_HEIGHT,
  };
  //PXP_SetCsc1Mode(APP_PXP, kPXP_Csc1YCbCr2RGB);/*!< YCbCr to RGB. */
  PXP_EnableCsc1(APP_PXP, false); //When set to logic 1, bypass is enabled and the output pixels will be in the YUV/YCbCr color space.
  
  const pxp_csc2_config_t csc2Config =
  {
    .mode = kPXP_Csc2RGB2YUV,
    .A1 = 0.299,
    .A2 = 0.587,
    .A3 = 0.114,
    .B1 = -0.147,
    .B2 = -0.298,
    .B3 = 0.436,
    .C1 = 0.615,
    .C2 = -0.151,
    .C3 = -0.100,
    .D1 = 0,
    .D2 = 0,
    .D3 = 0,
  };
  
  PXP_SetCsc2Config(APP_PXP, &csc2Config);  // RGB to YUV
  
  PXP_EnableCsc2(APP_PXP, true);
  /*
  * Configure the camera.
  */
  CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);
  
  CAMERA_DEVICE_Start(&cameraDevice);
  
  CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
  
  /* Submit the empty frame buffers to buffer queue. */
  for (uint32_t i = 0; i < APP_CAMERA_FRAME_BUFFER_COUNT; i++)
  {
    CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(s_cameraBuffer[i]));
  }
  
  CAMERA_RECEIVER_Start(&cameraReceiver);
  
  ELCDIF_RgbModeStart(APP_ELCDIF);
  
  while (1)
  {
    /* Wait to get the full frame buffer to show. */
    while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &fullCameraBufferAddr))
    {
    }
    
    /* Convert the camera input picture to RGB format. */
    psBufferConfig.bufferAddr = fullCameraBufferAddr;   /*!< Input buffer address for the first panel. */
    PXP_SetProcessSurfaceBufferConfig(APP_PXP, &psBufferConfig);
    
    //outputBufferConfig.buffer0Addr = (uint32_t)s_lcdBuffer[lcdInactiveFbIdx]; /*!< Output buffer 0 address. */
    outputBufferConfig.buffer0Addr = (uint32_t)decoderBuffer[DecoderInactiveFbIdx]; /*!< Output buffer 0 address. */
    PXP_SetOutputBufferConfig(APP_PXP, &outputBufferConfig);
    
    PXP_Start(APP_PXP);
    
    /* Wait for PXP process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(APP_PXP)))
    {
    }
    PXP_ClearStatusFlags(APP_PXP, kPXP_CompleteFlag);
    
    /**************************************************************************************/
#if 1	 //Decoder
    //For Bar/QR decoder
    cppResetBinarizer();//reset
    for(int cnt=0;cnt<APP_CAMERA_HEIGHT;cnt=cnt+8)   //  8 lines each
    {
      //Process every 8 lines in decoder
      memcpy(QR_BUFFER, &decoderBuffer[DecoderInactiveFbIdx][cnt], APP_CAMERA_WIDTH*8);	
      /* QR decoder feed data */
      cppProcessBlockRow(QR_BUFFER); //QR decoder process 8 lines
    } 
    
    /* load image data */
    set_martix_data(0, 0, APP_CAMERA_WIDTH, APP_CAMERA_HEIGHT, cppGetResults());	
    
    //test_results = cppGetResults();	
    
    ret = read_image(DECODE_1D, format, result); /* QR code */
    if(ret != 0)
    {
      ret = read_image(DECODE_QR, format, result); /* 1D code */
    }	
    
    if(ret == 0) //Decode success
    {
      PRINTF("Zxing Decode successfully\r\n");
      PRINTF("%s\r\n", format);
      PRINTF("%s\r\n", result);		
    }	
#endif	
    /**************************************************************************************/	
    
#if 1       //LCD Display
    ELCDIF_ClearInterruptStatus(APP_ELCDIF, kELCDIF_CurFrameDone);
    
    /* Set the new frame to LCD. */
    ELCDIF_SetNextBufferAddr(APP_ELCDIF,fullCameraBufferAddr);
    
    /* Wait for the new set LCD frame buffer active. */
    while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(APP_ELCDIF)))
    {
    }
    
#endif
    
    /* Return the camera buffer to camera queue. */
    CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, fullCameraBufferAddr);
    
    DecoderInactiveFbIdx ^= 1;
  }
}
