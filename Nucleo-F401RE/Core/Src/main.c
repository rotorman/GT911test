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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

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
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
	HAL_UART_Transmit(&huart2, (uint8_t *) t, strlen(t), 10);
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
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
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
	HAL_Delay(10);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 400000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TOUCH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_DMA_TRACE_OUT_GPIO_Port, I2C_DMA_TRACE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_IRQ_TRACE_OUT_GPIO_Port, I2C_IRQ_TRACE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TOUCH_RST_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TOUCH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_DMA_TRACE_OUT_Pin */
  GPIO_InitStruct.Pin = I2C_DMA_TRACE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_DMA_TRACE_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_IRQ_TRACE_OUT_Pin */
  GPIO_InitStruct.Pin = I2C_IRQ_TRACE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_IRQ_TRACE_OUT_GPIO_Port, &GPIO_InitStruct);
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
