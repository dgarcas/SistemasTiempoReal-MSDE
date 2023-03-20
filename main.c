/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include "dwt_stm32_delay.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t interruptionSemaphore = NULL;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);

void StartTarea1(void const *argument);
int ContTarea1 = 0;

#define PR_TAREA1 2
#define T_TAREA1 300
#define TRUE 1
#define FALSE 0

/******************************* Accelerometer ********************************/
double X, Y, Z;
int alturaReferencia;
int encenderTodosLosMotores = 0;

#define MOTOR_12 0
#define MOTOR_13 1
#define MOTOR_14 2
#define MOTOR_15 3
int estadoMotores[4] = { 0, 0, 0, 0 };
int estadoEstatico = 0;

//Vibraciones
int vibracionesDetectadas = 0;
double vibracionEjeX = 0;
double vibracionEjeY = 0;
double vibracionEjeZ = 0;

//Periodos
static int ALTITUD_TAREA_PERIODO_MS = 300;
static int ESTABILIZAR_TAREA_PREIODO_MS = 200;
static int CONTROL_MOTORES_TAREA_PERIDO_MS = 150;
static int DETECTAR_VIBRACIONES_TAREA_PERIODO_MS = 350;

uint8_t spiTxBuf[2], spiRxBuf[2];
uint8_t SPI_Read(uint8_t address);

void Init_Accelerometer(void);
void Obtain_Coordinates_XYZ(void);
double Calculate_RotationX(void);
double Calculate_RotationY(void);
void ajustarAltura(void *argument);
int Calculate_Hight(void);
void estabilizar(void *argument);
void estabilizarMotores(int motorA, int motorB, double rotacion);
void controlMotores(void *argument);
void manejarMotores(void);
void detectarVibraciones(void *argument);
void actualizarVibraciones(void);
int detectarVibracion(void);
void comprobarVibraciones(void);
void iniciarSistema(void *argument);
void encenderOApagarMotores();

int main(void) {

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_CAN1_Init();
	Init_Accelerometer();

	vibracionesDetectadas = 1;
	actualizarVibraciones();

	xSemaphore = xSemaphoreCreateMutex();
	interruptionSemaphore = xSemaphoreCreateBinary();
	xTaskCreate(ajustarAltura, "ajustarAltura", configMINIMAL_STACK_SIZE, NULL,
			1, NULL);
	xTaskCreate(estabilizar, "estabilizar", configMINIMAL_STACK_SIZE, NULL, 1,
	NULL);
	xTaskCreate(controlMotores, "controlMotores", configMINIMAL_STACK_SIZE,
	NULL, 1,
	NULL);
	xTaskCreate(detectarVibraciones, "detectarVibraciones",
	configMINIMAL_STACK_SIZE,
	NULL, 1,
	NULL);
	xTaskCreate(iniciarSistema, "iniciarSistema",
	configMINIMAL_STACK_SIZE,
	NULL, 1,
	NULL);
	vTaskStartScheduler();

	while (1) {
	}
}

void iniciarSistema(void *argument) {
	while (1) {
		xSemaphoreTake(interruptionSemaphore, portMAX_DELAY);
		alturaReferencia = Calculate_Hight();
		estadoEstatico = 1;
	}
}

void detectarVibraciones(void *argument) {
	TickType_t xLastWakeTime;

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		if (estadoEstatico == 1)
			comprobarVibraciones();

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(DETECTAR_VIBRACIONES_TAREA_PERIODO_MS));
	}
}

void comprobarVibraciones() {
	Obtain_Coordinates_XYZ();

	if (detectarVibracion() == 1) {
		vibracionesDetectadas++;
	} else {
		vibracionesDetectadas = 1;
	}

	if (vibracionesDetectadas >= 4) {

		estadoMotores[MOTOR_12] = 0;
		estadoMotores[MOTOR_13] = 0;
		estadoMotores[MOTOR_14] = 0;
		estadoMotores[MOTOR_15] = 0;

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

		estadoEstatico = 0;
		vibracionesDetectadas = 1;
	}

	actualizarVibraciones();
}

int detectarVibracion() {
	int vibracion = 0;

	if (fabs(fabs(X) - fabs(vibracionEjeX)) >= 0.05
			|| fabs(fabs(Y) - fabs(vibracionEjeY)) >= 0.05
			|| fabs(fabs(Z) - fabs(vibracionEjeZ)) >= 0.05) {
		vibracion = 1;
	}

	return vibracion;
}

void actualizarVibraciones() {
	vibracionEjeX = X;
	vibracionEjeY = Y;
	vibracionEjeZ = Z;
}

void controlMotores(void *argument) {

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		if (estadoEstatico == 1)
			manejarMotores();

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(CONTROL_MOTORES_TAREA_PERIDO_MS));
	}
}

void manejarMotores() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, estadoMotores[MOTOR_12]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, estadoMotores[MOTOR_13]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, estadoMotores[MOTOR_14]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, estadoMotores[MOTOR_15]);
}

void estabilizar(void *argument) {

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		if (estadoEstatico == 1) {
			estabilizarMotores(MOTOR_13, MOTOR_15, Calculate_RotationX());
			estabilizarMotores(MOTOR_12, MOTOR_14, Calculate_RotationY());
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(ESTABILIZAR_TAREA_PREIODO_MS));
	}
}

void estabilizarMotores(int motorA, int motorB, double rotacion) {

	int estadoMotorA, estadoMotorB;

	if (rotacion < -10) {
		estadoMotorA = 0;
		estadoMotorB = 1;

	} else if (rotacion > 10) {
		estadoMotorA = 1;
		estadoMotorB = 0;
	} else {
		estadoMotorA = 0;
		estadoMotorB = 0;
	}

	estadoMotores[motorA] = estadoMotorA;
	estadoMotores[motorB] = estadoMotorB;

}

void ajustarAltura(void *argument) {
	TickType_t xLastWakeTime;

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		if (estadoEstatico == 1)
			encenderOApagarMotores();

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(ALTITUD_TAREA_PERIODO_MS));

	}
}

void encenderOApagarMotores() {
	int alturaActual = Calculate_Hight();
	if ((alturaReferencia - alturaActual) >= 5) {
		estadoMotores[MOTOR_12] = 1;
		estadoMotores[MOTOR_13] = 1;
		estadoMotores[MOTOR_14] = 1;
		estadoMotores[MOTOR_15] = 1;
	} else {
		estadoMotores[MOTOR_12] = 0;
		estadoMotores[MOTOR_13] = 0;
		estadoMotores[MOTOR_14] = 0;
		estadoMotores[MOTOR_15] = 0;
	}
}

int Calculate_Hight() {

	int actual = 0;

	ADC_ChannelConfTypeDef sConfigN = { 0 };
	sConfigN.Channel = ADC_CHANNEL_0;
	sConfigN.Rank = 1;
	sConfigN.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	HAL_ADC_ConfigChannel(&hadc1, &sConfigN);
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
		actual = HAL_ADC_GetValue(&hadc1);
	}
	return actual;
}

void Init_Accelerometer() {

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	spiTxBuf[0] = 0x20;
	spiTxBuf[1] = 0x17;
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	spiTxBuf[0] = 0x20 | 0x80;
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
	HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

uint8_t SPI_Read(uint8_t address) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	spiTxBuf[0] = address | 0x80;
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
	HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return spiRxBuf[0];
}

/**
 * Function body to obtain the coordinates of the three accelerometer axes
 */
void Obtain_Coordinates_XYZ() {
	int Ix, Iy, Iz;
	uint8_t Ix1, Ix2;
	uint8_t Iy1, Iy2;
	uint8_t Iz1, Iz2;

	Ix1 = SPI_Read(0x28);
	Ix2 = SPI_Read(0x29);
	Ix = (Ix2 << 8) + Ix1;
	if (Ix >= 0x8000)
		Ix = -(65536 - Ix);
	X = Ix / 16384.0;

	Iy1 = SPI_Read(0x2A);
	Iy2 = SPI_Read(0x2B);
	Iy = (Iy2 << 8) + Iy1;
	if (Iy >= 0x8000)
		Iy = -(65536 - Iy);
	Y = Iy / 16384.0;

	Iz1 = SPI_Read(0x2C);
	Iz2 = SPI_Read(0x2D);
	Iz = (Iz2 << 8) + Iz1;
	if (Iz >= 0x8000)
		Iz = -(65536 - Iz);
	Z = Iz / 16384.0;
}

double Calculate_RotationX() {
	double rotX;

	Obtain_Coordinates_XYZ();
	rotX = atan2(Y, sqrt(X * X + Z * Z)) * 180.0 / 3.1416;

	return rotX;
}

double Calculate_RotationY() {
	double rotY;

	Obtain_Coordinates_XYZ();
	rotY = -atan2(X, sqrt(Y * Y + Z * Z)) * 180.0 / 3.1416;

	return rotY;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig = { 0 };

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 21;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB3: Interrupcion botones externos */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PD10 PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 |
	GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PD11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 2,
			0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1,
			0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/**
 * Funcion para el tratamiento de interrupciones
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	long yield = pdFALSE;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//  /* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
	portYIELD_FROM_ISR(yield);
	xSemaphoreGiveFromISR(interruptionSemaphore, &xHigherPriorityTaskWoken);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
