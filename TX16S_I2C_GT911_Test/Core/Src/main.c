/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t track ;
	uint16_t x ;
	uint16_t y ;
	uint16_t size ;
	uint8_t reserved ;
} TouchPoint;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GT911_I2C_ADDR					0x14
#define GT911_MAX_TP					5
#define GT911_CFG_NUMER					0x6C
#define GT911_TIMEOUT					3 // 3ms

#define GT_CTRL_REG 					0x8040
#define GT_CFGS_REG 					0x8047
#define GT_CHECK_REG 					0x80FF
#define GT_PID_REG 						0x8140

#define GT_GSTID_REG 					0x814E
#define GT_TP1_REG 						0x8150
#define GT_TP2_REG 						0x8158
#define GT_TP3_REG 						0x8160
#define GT_TP4_REG 						0x8168
#define GT_TP5_REG 						0x8170

#define GT911_READ_XY_REG               0x814E
#define GT911_CLEARBUF_REG              0x814E
#define GT911_CONFIG_REG                0x8047
#define GT911_COMMAND_REG               0x8040
#define GT911_PRODUCT_ID_REG            0x8140
#define GT911_VENDOR_ID_REG             0x814A
#define GT911_CONFIG_VERSION_REG        0x8047
#define GT911_CONFIG_CHECKSUM_REG       0x80FF
#define GT911_FIRMWARE_VERSION_REG      0x8144

#define TPRST_LOW()   HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_RESET)
#define TPRST_HIGH()  HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_SET)

#define TPINT_LOW()   HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_RESET)
#define TPINT_HIGH()  HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_SET)

//GT911 param table
const uint8_t TOUCH_GT911_Cfg[] =
{
	GT911_CFG_NUMER,// 0x8047 Config version
	0xE0,			// 0x8048 X output map : x 480
	0x01,
	0x10,			// 0x804A Y ouptut max : y 272
	0x01,
	GT911_MAX_TP,	// 0x804C Touch number
	0x3C,			// 0x804D Module switch 1 : bit4= xy change Int mode
	0x20,			// 0x804E Module switch 2
	0x22,			// 0x804F Shake_Count
	0x0A,			// 0x8050 Filter
	0x28,			// 0x8051 Larger touch
	0x0F,			// 0x8052 Noise reduction
	0x5A,			// 0x8053 Screen touch level
	0x3C,			// 0x8054 Screen touch leave
	0x03,			// 0x8055 Low power control
	0x0F,			// 0x8056 Refresh rate
	0x01,			// 0x8057 X threshold
	0x01,			// 0x8058 Y threshold
	0x00,			// 0x8059 Reserved
	0x00,			// 0x805A Reserved
	0x11,			// 0x805B Space (top, bottom)
	0x11,			// 0x805C Space (left, right)
	0x08,			// 0x805D Mini filter
	0x18,			// 0x805E Strech R0
	0x1A,			// 0x805F Strech R1
	0x1E,			// 0x8060 Strech R2
	0x14,			// 0x8061 Strech RM
	0x87,			// 0x8062 Drv groupA num
	0x29,			// 0x8063 Drv groupB num
	0x0A,			// 0x8064 Sensor num
	0xCF,			// 0x8065 FreqA factor
	0xD1,			// 0x8066 FreqB factor
	0xB2,			// 0x8067 Panel bit freq
	0x04,
	0x00,			// 0x8069 Reserved
	0x00,
	0x00,			// 0x806B Panel tx gain
	0xD8,			// 0x806C Panel rx gain
	0x02,			// 0x806D Panel dump shift
	0x1D,			// 0x806E Drv frame control
	0x00,			// 0x806F Charging level up
	0x01,			// 0x8070 Module switch 3
	0x00,			// 0x8071 Gesture distance
	0x00,			// 0x8072 Gesture long press time
	0x00,			// 0x8073 X/Y slope adjust
	0x00,			// 0x8074 Gesture control
	0x00,			// 0x8075 Gesture switch 1
	0x00,			// 0x8076 Gesture switch 2
	0x00,			// 0x8077 Gesture refresh rate
	0x00,			// 0x8078 Gesture touch level
	0x00,			// 0x8079 New green wake up level
	0xB4,			// 0x807A Freq hopping start
	0xEF,			// 0x807B Freq hopping end
	0x94,			// 0x807C Noise detect time
	0xD5,			// 0x807D Hopping flag
	0x02,			// 0x807E Hopping flag
	0x07,			// 0x807F Noise threshold
	0x00,			// 0x8080 Nois min threshold old
	0x00,			// 0x8081 Reserved
	0x04,			// 0x8082 Hopping sensor group
	0x6E,			// 0x8083 Hopping Seg1 normalize
	0xB9,			// 0x8084 Hopping Seg1 factor
	0x00,			// 0x8085 Main clock adjust
	0x6A,			// 0x8086 Hopping Seg2 normalize
	0xC4,			// 0x8087 Hopping Seg2 factor
	0x00,			// 0x8088 Reserved
	0x66,			// 0x8089 Hopping Seg3 normalize
	0xCF,			// 0x808A Hopping Seg3 factor
	0x00,			// 0x808B Reserved
	0x62,			// 0x808C Hopping Seg4 normalize
	0xDB,			// 0x808D Hopping Seg4 factor
	0x00,			// 0x808E Reserved
	0x5E,			// 0x808F Hopping Seg5 normalize
	0xE8,			// 0x8090 Hopping Seg5 factor
	0x00,			// 0x8091 Reserved
	0x5E,			// 0x8092 Hopping Seg6 normalize
	0x00,			// 0x8093 Key1
	0x00,			// 0x8094 Key2
	0x00,			// 0x8095 Key3
	0x00,			// 0x8096 Key4
	0x00,			// 0x8097 Key area
	0x00,			// 0x8098 Key touch level
	0x00,			// 0x8099 Key leave level
	0x00,			// 0x809A Key sens
	0x00,			// 0x809B Key sens
	0x00,			// 0x809C Key restrain
	0x00,			// 0x809D Key restrain time
	0x00,			// 0x809E Large gesture touch
	0x00,			// 0x809F Reserved
	0x00,			// 0x80A0 Reserved
	0x00,			// 0x80A1 Hotknot noise map
	0x00,			// 0x80A2 Link threshold
	0x00,			// 0x80A3 Pxy threshold
	0x00,			// 0x80A4 GHot dump shift
	0x00,			// 0x80A5 GHot rx gain
	0x00,			// 0x80A6 Freg gain
	0x00,			// 0x80A7 Freg gain 1
	0x00,			// 0x80A8 Freg gain 2
	0x00,			// 0x80A9 Freg gain 3
	0x00,			// 0x80AA Reserved
	0x00,			// 0x80AB Reserved
	0x00,			// 0x80AC Reserved
	0x00,			// 0x80AD Reserved
	0x00,			// 0x80AE Reserved
	0x00,			// 0x80AF Reserved
	0x00,			// 0x80B0 Reserved
	0x00,			// 0x80B1 Reserved
	0x00,			// 0x80B2 Reserved
	0x00,			// 0x80B3 Combine dis
	0x00,			// 0x80B4 Split set
	0x00,			// 0x80B5 Reserved
	0x00,			// 0x80B6 Reserved
	0x14,			// 0x80B7 Sensor CH0
	0x12,			// 0x80B8 Sensor CH1
	0x10,			// 0x80B9 Sensor CH2
	0x0E,			// 0x80BA Sensor CH3
	0x0C,			// 0x80BB Sensor CH4
	0x0A,			// 0x80BC Sensor CH5
	0x08,			// 0x80BD Sensor CH6
	0x06,			// 0x80BE Sensor CH7
	0x04,			// 0x80BF Sensor CH8
	0x02,			// 0x80C0 Sensor CH9
	0xFF,			// 0x80C1 Sensor CH10
	0xFF,			// 0x80C2 Sensor CH11
	0xFF,			// 0x80C3 Sensor CH12
	0xFF,			// 0x80C4 Sensor CH13
	0x00,			// 0x80C5 Reserved
	0x00,			// 0x80C6 Reserved
	0x00,			// 0x80C7 Reserved
	0x00,			// 0x80C8 Reserved
	0x00,			// 0x80C9 Reserved
	0x00,			// 0x80CA Reserved
	0x00,			// 0x80CB Reserved
	0x00,			// 0x80CC Reserved
	0x00,			// 0x80CD Reserved
	0x00,			// 0x80CE Reserved
	0x00,			// 0x80CF Reserved
	0x00,			// 0x80D0 Reserved
	0x00,			// 0x80D1 Reserved
	0x00,			// 0x80D2 Reserved
	0x00,			// 0x80D3 Reserved
	0x00,			// 0x80D4 Reserved
	0x28,			// 0x80D5 Driver CH0
	0x26,			// 0x80D6 Driver CH1
	0x24,			// 0x80D7 Driver CH2
	0x22,			// 0x80D8 Driver CH3
	0x21,			// 0x80D9 Driver CH4
	0x20,			// 0x80DA Driver CH5
	0x1F,			// 0x80DB Driver CH6
	0x1E,			// 0x80DC Driver CH7
	0x1D,			// 0x80DD Driver CH8
	0x0C,			// 0x80DE Driver CH9
	0x0A,			// 0x80DF Driver CH10
	0x08,			// 0x80E0 Driver CH11
	0x06,			// 0x80E1 Driver CH12
	0x04,			// 0x80E2 Driver CH13
	0x02,			// 0x80E3 Driver CH14
	0x00,			// 0x80E4 Driver CH15
	0xFF,			// 0x80E5 Driver CH16
	0xFF,			// 0x80E6 Driver CH17
	0xFF,			// 0x80E7 Driver CH18
	0xFF,			// 0x80E8 Driver CH19
	0xFF,			// 0x80E9 Driver CH20
	0xFF,			// 0x80EA Driver CH21
	0xFF,			// 0x80EB Driver CH22
	0xFF,			// 0x80EC Driver CH23
	0xFF,			// 0x80ED Driver CH24
	0xFF,			// 0x80EE Driver CH25
	0x00,			// 0x80EF Reserved
	0x00,			// 0x80F0 Reserved
	0x00,			// 0x80F1 Reserved
	0x00,			// 0x80F2 Reserved
	0x00,			// 0x80F3 Reserved
	0x00,			// 0x80F4 Reserved
	0x00,			// 0x80F5 Reserved
	0x00,			// 0x80F6 Reserved
	0x00,			// 0x80F7 Reserved
	0x00,			// 0x80F8 Reserved
	0x00,			// 0x80F9 Reserved
	0x00,			// 0x80FA Reserved
	0x00,			// 0x80FB Reserved
	0x00,			// 0x80FC Reserved
	0x00,			// 0x80FD Reserved
	0x00			// 0x80FE Reserved
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct TouchData {
	union
	{
		TouchPoint points[GT911_MAX_TP] ;
		uint8_t data[GT911_MAX_TP * sizeof(TouchPoint)] ;
	};
};

enum TouchEvent
{
	TE_NONE,
	TE_DOWN,
	TE_UP,
	TE_SLIDE,
	TE_SLIDE_END
};

struct TouchState
{
	unsigned char event;
	short x;
	short y;
	short startX;
	short startY;
	short deltaX;
	short deltaY;
	short lastDeltaX;
	short lastDeltaY;
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint16_t touchGT911fwver = 0;
uint16_t touchGT911hiccups = 0;
struct TouchData touchData;
struct TouchState touchState;
const uint8_t SLIDE_RANGE = 6;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_FMC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_LTDC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TRACE_TIME_FORMAT     "%0.2f "
#define TRACE_TIME_VALUE      ((float)HAL_GetTick())
#define CRLF "\r\n"

#define PRINTF_BUFFER_SIZE    128

void serialPrintf(const char * format, ...)
{
	va_list arglist;
	char tmp[PRINTF_BUFFER_SIZE + 1];

	snprintf(tmp, PRINTF_BUFFER_SIZE, "+%05lums: ", HAL_GetTick());
	va_start(arglist, format);
	vsnprintf(tmp + strlen(tmp), PRINTF_BUFFER_SIZE - strlen(tmp), format, arglist);
	tmp[PRINTF_BUFFER_SIZE] = '\0';
	va_end(arglist);

	const char *t = tmp;
	HAL_UART_Transmit(&huart3, (uint8_t *) t, strlen(t), 10);
}

#define debugPrintf(...) do { serialPrintf(__VA_ARGS__); } while(0)
#define TRACE(f_, ...)        debugPrintf((TRACE_TIME_FORMAT f_ CRLF), TRACE_TIME_VALUE, ##__VA_ARGS__)

void TOUCH_AF_INT_Change(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = TOUCH_INT_Pin;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStructure);
}

static void TOUCH_AF_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = TOUCH_RST_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_RST_GPIO_Port, &GPIO_InitStructure);

	HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_RESET);

	GPIO_InitStructure.Pin = TOUCH_INT_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStructure);

	HAL_GPIO_WritePin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin, GPIO_PIN_RESET);
}

bool I2C_GT911_ReadRegister(uint16_t reg, uint8_t * buf, uint8_t len)
{
	uint8_t uRegAddr[2];
	uRegAddr[0] = (uint8_t)((reg & 0xFF00) >> 8);
	uRegAddr[1] = (uint8_t)(reg & 0x00FF);

	if (HAL_I2C_Master_Transmit(&hi2c1, GT911_I2C_ADDR << 1, uRegAddr, 2, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: ReadRedister write reg address failed");
		asm("bkpt 255");
		return false;
	}

	if (HAL_I2C_Master_Receive(&hi2c1, GT911_I2C_ADDR << 1, buf, len, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: ReadRedister read reg address failed");
		asm("bkpt 255");
		return false;
	}
	return true;
}

uint8_t I2C_GT911_WriteRegister(uint16_t reg, uint8_t * buf, uint8_t len)
{
	uint8_t uAddrAndBuf[258];
	uAddrAndBuf[0] = (uint8_t)((reg & 0xFF00) >> 8);
	uAddrAndBuf[1] = (uint8_t)(reg & 0x00FF);

	if (len > 0)
	{
		for (int i = 0;i < len;i++)
		{
			uAddrAndBuf[i + 2] = buf[i];
		}
	}

	if (HAL_I2C_Master_Transmit(&hi2c1, GT911_I2C_ADDR << 1, uAddrAndBuf, len + 2, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: WriteRegister failed");
		asm("bkpt 255");
		return false;
	}
	return true;
}

bool I2C_GT911_SendConfig(void)
{
	uint8_t buf[2];
	uint8_t i = 0;
	buf[0] = 0;
	buf[1] = 1;
	bool bResult = true;

	for (i = 0; i < sizeof(TOUCH_GT911_Cfg); i++)
		buf[0] += TOUCH_GT911_Cfg[i];//check sum

	buf[0] = (~buf[0]) + 1;
	if (!I2C_GT911_WriteRegister(GT_CFGS_REG, (uint8_t *) TOUCH_GT911_Cfg, sizeof(TOUCH_GT911_Cfg)))
	{
		TRACE("GT911 ERROR: write config failed");
		bResult = false;
	}

	if (!I2C_GT911_WriteRegister(GT_CHECK_REG, buf, 2)) //write checksum
		{
			TRACE("GT911 ERROR: write config checksum failed");
			bResult = false;
		}
	return bResult;
}

bool touchPanelInit(void)
{
	uint8_t tmp[4] = { 0 };

	TRACE("Touchpanel init start ...");

	TOUCH_AF_GPIOConfig(); //SET RST=OUT INT=OUT INT=LOW
	//I2C_Init();

	TPRST_LOW();
	TPINT_HIGH();
	HAL_Delay(1);

	TPRST_HIGH();
	HAL_Delay(6);

	TPINT_LOW();
	HAL_Delay(55);

	TOUCH_AF_INT_Change();  //Set INT INPUT INT=LOW

	HAL_Delay(50);

	TRACE("Reading Touch registry");
	if (!I2C_GT911_ReadRegister(GT_PID_REG, tmp, 4))
	{
		TRACE("GT911 ERROR: Product ID read failed");
	}

	if (strcmp((char *) tmp, "911") == 0)
	{
		TRACE("GT911 chip detected");
		tmp[0] = 0X02;
		if (!I2C_GT911_WriteRegister(GT_CTRL_REG, tmp, 1))
		{
			TRACE("GT911 ERROR: write to control register failed");
		}
		if (!I2C_GT911_ReadRegister(GT_CFGS_REG, tmp, 1))
		{
			TRACE("GT911 ERROR: configuration register read failed");
		}

		TRACE("Chip config Ver:%x", tmp[0]);
		if (tmp[0] <= GT911_CFG_NUMER)  //Config ver
		{
			TRACE("Sending new config %d", GT911_CFG_NUMER);
			if (!I2C_GT911_SendConfig())
			{
				TRACE("GT911 ERROR: sending configration failed");
			}
		}

		if (!I2C_GT911_ReadRegister(GT911_FIRMWARE_VERSION_REG, tmp, 2))
		{
			TRACE("GT911 ERROR: reading firmware version failed");
		}
		else
		{
			touchGT911fwver = (tmp[1] << 8) + tmp[0];
			TRACE("GT911 FW version: %u", touchGT911fwver);
		}

		HAL_Delay(10);
		tmp[0] = 0X00;
		if (!I2C_GT911_WriteRegister(GT_CTRL_REG, tmp, 1))  //end reset
		{
			TRACE("GT911 ERROR: write to control register failed");
		}
		// touchGT911Flag = true;

		//TOUCH_AF_ExtiConfig();

		return true;
	}
	TRACE("GT911 chip NOT FOUND");
	return false;
}

void touchPanelDeInit(void)
{
	//TOUCH_AF_ExtiStop();
	//touchGT911Flag = false;
	TRACE("touchPanelDeInit()");
	asm("bkpt 255");
}

void touchPanelRead()
{
	uint8_t state = 0;

	// if (!touchEventOccured)
	//   return;

	// touchEventOccured = false;

	uint32_t startReadStatus = HAL_GetTick();
	do {
		if (!I2C_GT911_ReadRegister(GT911_READ_XY_REG, &state, 1)) {
			HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_SET);
			touchGT911hiccups++;
			TRACE("GT911 I2C read XY error");
			touchPanelDeInit();
			touchPanelInit();
			return;
		}

		if (state & 0x80u) {
			// ready
			break;
		}
		HAL_Delay(1);
	} while (HAL_GetTick() - startReadStatus < GT911_TIMEOUT);

	TRACE("touch state = 0x%x", state);
	if (state & 0x80u) {
		uint8_t pointsCount = (state & 0x0Fu);

		if (pointsCount > 0 && pointsCount <= GT911_MAX_TP) {
			if (!I2C_GT911_ReadRegister(GT911_READ_XY_REG + 1, touchData.data, pointsCount * sizeof(TouchPoint)))
			{
				HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_SET);
				touchGT911hiccups++;
				TRACE("GT911 I2C data read error");
				touchPanelDeInit();
				touchPanelInit();
				return;
			}
			if (touchState.event == TE_NONE || touchState.event == TE_UP || touchState.event == TE_SLIDE_END)
			{
				touchState.event = TE_DOWN;
				touchState.startX = touchState.x = touchData.points[0].x;
				touchState.startY = touchState.y = touchData.points[0].y;
			}
			else
			{
				touchState.deltaX = touchData.points[0].x - touchState.x;
				touchState.deltaY = touchData.points[0].y - touchState.y;
				if (touchState.event == TE_SLIDE || abs(touchState.deltaX) >= SLIDE_RANGE || abs(touchState.deltaY) >= SLIDE_RANGE)
				{
					touchState.event = TE_SLIDE;
					touchState.x = touchData.points[0].x;
					touchState.y = touchData.points[0].y;
				}
			}
		}
		else
		{
			if (touchState.event == TE_SLIDE)
			{
				touchState.event = TE_SLIDE_END;
			}
			else if (touchState.event == TE_DOWN)
			{
				touchState.event = TE_UP;
			}
			else if (touchState.event != TE_SLIDE_END) {
				touchState.event = TE_NONE;
			}
		}
	}

	uint8_t zero = 0;
	if (!I2C_GT911_WriteRegister(GT911_READ_XY_REG, &zero, 1))
	{
		TRACE("GT911 ERROR: clearing XY register failed");
	}

	switch (touchState.event) {
		case TE_NONE:
			TRACE("touch event = NONE"); break;
		case TE_UP:
			TRACE("touch event = UP"); break;
		case TE_DOWN:
			TRACE("touch event = DOWN"); break;
		case TE_SLIDE_END:
			TRACE("touch event = SLIDE_END"); break;
		case TE_SLIDE:
			TRACE("touch event = SLIDE"); break;
		default:
			TRACE("touch event = UNKNOWN"); break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FMC_Init();
  MX_USART3_UART_Init();
  MX_LTDC_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_SET); // Turn on power
  HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDgreen_GPIO_Port, LEDgreen_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  if (!touchPanelInit())
  {
	  TRACE("ERROR: touchPanelInit() failed");
	  asm("bkpt 255");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GPIO_ReadPin(TOUCH_INT_GPIO_Port, TOUCH_INT_Pin) == GPIO_PIN_SET)
	  {
		  touchPanelRead();
	  }



	  // Check Power-Off
	  if (HAL_GPIO_ReadPin(PWRswitch_GPIO_Port, PWRswitch_Pin) == GPIO_PIN_RESET)
	  {
		  //HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_SET);
		  //HAL_GPIO_WritePin(LEDgreen_GPIO_Port, LEDgreen_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_SET);

		  // Check again in 1 second
		  HAL_Delay(1000);
		  if (HAL_GPIO_ReadPin(PWRswitch_GPIO_Port, PWRswitch_Pin) == GPIO_PIN_RESET)
		  {
			  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_RESET); // Turn off power
		  }
	  }
	  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_RESET);

	  HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 2;
  hltdc.Init.VerticalSync = 10;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 12;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 284;
  hltdc.Init.TotalWidth = 525;
  hltdc.Init.TotalHeigh = 286;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 479;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 271;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 261120;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 479;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 271;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 261120;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 400000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEDred_Pin|LEDgreen_Pin|LEDblue_Pin|HAPTIC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INTMODboot_GPIO_Port, INTMODboot_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCDnRST_GPIO_Port, LCDnRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TESTPOINT_Pin|IntModPwr_Pin|UART3Pwr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AudioMute_GPIO_Port, AudioMute_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UART6pwr_Pin|LCDbacklight_Pin|ExtModPwr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TrainerOut_GPIO_Port, TrainerOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TelemDir_GPIO_Port, TelemDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BluetoothEn_GPIO_Port, BluetoothEn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDred_Pin LEDgreen_Pin LEDblue_Pin HAPTIC_Pin */
  GPIO_InitStruct.Pin = LEDred_Pin|LEDgreen_Pin|LEDblue_Pin|HAPTIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SWEL_Pin */
  GPIO_InitStruct.Pin = SWEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYenter_Pin KEYpageprevious_Pin SWAL_Pin KEYrtn_Pin
                           KEYtelem_Pin KEYmdl_Pin KEYsys_Pin */
  GPIO_InitStruct.Pin = KEYenter_Pin|KEYpageprevious_Pin|SWAL_Pin|KEYrtn_Pin
                          |KEYtelem_Pin|KEYmdl_Pin|KEYsys_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYpagenext_Pin TrimLHR_Pin */
  GPIO_InitStruct.Pin = KEYpagenext_Pin|TrimLHR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INTMODboot_Pin LCDnRST_Pin */
  GPIO_InitStruct.Pin = INTMODboot_Pin|LCDnRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_RST_Pin */
  GPIO_InitStruct.Pin = TOUCH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOUCH_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_INT_Pin PCBREV1_Pin PCBREV2_Pin ROTENCB_Pin
                           ROTENCA_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin|PCBREV1_Pin|PCBREV2_Pin|ROTENCB_Pin
                          |ROTENCA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : SWF_Pin SWEH_Pin SWAH_Pin SWBH_Pin
                           SWI_Pin SWJ_Pin */
  GPIO_InitStruct.Pin = SWF_Pin|SWEH_Pin|SWAH_Pin|SWBH_Pin
                          |SWI_Pin|SWJ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : TESTPOINT_Pin AudioMute_Pin IntModPwr_Pin UART3Pwr_Pin */
  GPIO_InitStruct.Pin = TESTPOINT_Pin|AudioMute_Pin|IntModPwr_Pin|UART3Pwr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TrimLHL_Pin */
  GPIO_InitStruct.Pin = TrimLHL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TrimLHL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDpresent_Pin TrainerIn_Pin */
  GPIO_InitStruct.Pin = SDpresent_Pin|TrainerIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UART6pwr_Pin LCDbacklight_Pin ExtModPwr_Pin */
  GPIO_InitStruct.Pin = UART6pwr_Pin|LCDbacklight_Pin|ExtModPwr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWRswitch_Pin */
  GPIO_InitStruct.Pin = PWRswitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWRswitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWRon_Pin */
  GPIO_InitStruct.Pin = PWRon_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWRon_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWBL_Pin TrainerDetect_Pin */
  GPIO_InitStruct.Pin = SWBL_Pin|TrainerDetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TrimRSD_Pin TrimRSU_Pin SWCL_Pin */
  GPIO_InitStruct.Pin = TrimRSD_Pin|TrimRSU_Pin|SWCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWCH_Pin TrimLSU_Pin TrimR_Pin TrimRHR_Pin */
  GPIO_InitStruct.Pin = SWCH_Pin|TrimLSU_Pin|TrimR_Pin|TrimRHR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : HEARTBEAT_Pin */
  GPIO_InitStruct.Pin = HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWDH_Pin TrimLSD_Pin TrimRV_Pin TrimRVD_Pin
                           TrimLVU_Pin */
  GPIO_InitStruct.Pin = SWDH_Pin|TrimLSD_Pin|TrimRV_Pin|TrimRVD_Pin
                          |TrimLVU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : SWDL_Pin SWGL_Pin SWGH_Pin SWH_Pin
                           TrimLVD_Pin */
  GPIO_InitStruct.Pin = SWDL_Pin|SWGL_Pin|SWGH_Pin|SWH_Pin
                          |TrimLVD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : TrainerOut_Pin */
  GPIO_InitStruct.Pin = TrainerOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TrainerOut_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ExtModTX_Pin */
  GPIO_InitStruct.Pin = ExtModTX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ExtModTX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TelemDir_Pin */
  GPIO_InitStruct.Pin = TelemDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TelemDir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BluetoothEn_Pin */
  GPIO_InitStruct.Pin = BluetoothEn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BluetoothEn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USBchaCtrl_Pin USBchgDetect_Pin */
  GPIO_InitStruct.Pin = USBchaCtrl_Pin|USBchgDetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
