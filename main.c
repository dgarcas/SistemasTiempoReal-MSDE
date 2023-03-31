/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include "dwt_stm32_delay.h"
#include "estados.c"

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
int iteracionesSinEjecutarAltura = 0;

#define ITERACIONES_MAXIMA 10
#define MOTOR_Y_NEGATIVO 0 // Y Negativo
#define MOTOR_X_NEGATIVO 1 // X Negativo
#define MOTOR_Y_POSITIVO 2 // Y postivo
#define MOTOR_X_POSITIVO 3 // X postivo
int estadoDeseadoMotores[4] = { 0, 0, 0, 0 };
int encenderMotores = 0;
enum ESTADOS estadoGlobal = inicio;
enum HORIZONTALIDAD_ESTADOS estadoMotores = NNNN;

//Vibraciones
int vibracionesDetectadas = 0;
double vibracionEjeX = 0;
double vibracionEjeY = 0;
double vibracionEjeZ = 0;

//Contadores
int contadorCorrectorAltitud = 0;
int contadorCorrectorEstabilidad = 0;

//Periodos
static int ALTITUD_TAREA_PERIODO_MS = 300;
static int ESTABILIZAR_TAREA_PREIODO_MS = 200;
static int CONTROL_MOTORES_TAREA_PERIDO_MS = 150;
static int DETECTAR_VIBRACIONES_TAREA_PERIODO_MS = 350;
static int INICIO_SISTEMA_TAREA_PERIODO_MS = 500;
static int COMUNICACION_BASE_TAREA_PERIODO_MS = 2000;

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
void manejarMotoresEstabilidad(void);
void manejarMotoresAltura(void);
void detectarVibraciones(void *argument);
void actualizarVibraciones(void);
int detectarVibracion(void);
void comprobarVibraciones(void);
void iniciarSistema(void *argument);
void encenderOApagarMotores();
void comunicacionBase(void *argument);
void enviarDatos();
enum HORIZONTALIDAD_ESTADOS getEstadoDeseado();
void manejarMotores(int estadoDeMotores[]);

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
	xTaskCreate(comunicacionBase, "comunicacionBase",
	configMINIMAL_STACK_SIZE,
	NULL, 1,
	NULL);
	vTaskStartScheduler();

	while (1) {
	}
}

void comunicacionBase(void *argument) {
	TickType_t xLastWakeTime;

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		enviarDatos();

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(COMUNICACION_BASE_TAREA_PERIODO_MS));
	}
}

void enviarDatos() {

	printf("Correctiones altitud realizadas: %i\n", contadorCorrectorAltitud);
	printf("Correctiones estabilidad realizadas: %i\n",
			contadorCorrectorEstabilidad);
	printf("Altitud actual: %i\n", Calculate_Hight());
}

void iniciarSistema(void *argument) {
	TickType_t xLastWakeTime;
	while (1) {

		xSemaphoreTake(interruptionSemaphore, portMAX_DELAY);

		if (estadoGlobal == inicio) {
			alturaReferencia = Calculate_Hight();
			estadoGlobal = estatico;
		} else if (estadoGlobal == vuelta) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
			estadoGlobal = estatico;
		}
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(INICIO_SISTEMA_TAREA_PERIODO_MS));
	}
}

void detectarVibraciones(void *argument) {
	TickType_t xLastWakeTime;

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

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

		estadoDeseadoMotores[MOTOR_Y_NEGATIVO] = 0;
		estadoDeseadoMotores[MOTOR_X_NEGATIVO] = 0;
		estadoDeseadoMotores[MOTOR_Y_POSITIVO] = 0;
		estadoDeseadoMotores[MOTOR_X_POSITIVO] = 0;

		if (estadoGlobal != inicio) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

			estadoGlobal = vuelta;
		}

		vibracionesDetectadas = 1;
	}

	actualizarVibraciones();
}

int detectarVibracion() {
	int vibracion = 0;

	if (fabs(fabs(X) - fabs(vibracionEjeX)) >= 0.02
			|| fabs(fabs(Y) - fabs(vibracionEjeY)) >= 0.02
			|| fabs(fabs(Z) - fabs(vibracionEjeZ)) >= 0.02) {
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

		if (estadoGlobal == estatico || estadoGlobal == accenso) {
			manejarMotoresEstabilidad();
			if ((estadoMotores == NNNN)
					|| (iteracionesSinEjecutarAltura >= ITERACIONES_MAXIMA)) {
				manejarMotoresAltura();
				estadoMotores = NNNN;
				iteracionesSinEjecutarAltura = 0;
			} else if (estadoGlobal == accenso) {
				iteracionesSinEjecutarAltura++;
			}
		}

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(CONTROL_MOTORES_TAREA_PERIDO_MS));
	}
}

void manejarMotoresAltura() {
	if (!encenderMotores) {
		int estadoDeMotores[4] = { 0, 0, 0, 0 };
		manejarMotores(estadoDeMotores);
		estadoGlobal = estatico;
	} else {
		int estadoDeMotores[4] = { 1, 1, 1, 1 };
		manejarMotores(estadoDeMotores);
		estadoGlobal = accenso;
	}
}

void manejarMotoresEstabilidad() {

	enum HORIZONTALIDAD_ESTADOS estadoDeseado = getEstadoDeseado();

	if (estadoMotores == NNNN) {
		int estadoDeMotores[4];
		switch (estadoDeseado) {

		case NNNN:
			//if (estadoGlobal != accenso) {
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 0;
			//}

			break;
		case NNPN:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 1;
			estadoDeMotores[3] = 0;
			break;
		case NNPP:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 1;
			estadoDeMotores[3] = 1;
			break;
		case NNNP:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 1;
			break;
		case PNNP:
			estadoDeMotores[0] = 1;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 1;
			break;
		case PNNN:
			estadoDeMotores[0] = 1;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 0;
			break;
		case PPNN:
			estadoDeMotores[0] = 1;
			estadoDeMotores[1] = 1;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 0;
			break;
		case NPNN:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 1;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 0;
			break;
		case NPPN:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 1;
			estadoDeMotores[2] = 1;
			estadoDeMotores[3] = 0;
			break;
		default:
			estadoDeMotores[0] = 0;
			estadoDeMotores[1] = 0;
			estadoDeMotores[2] = 0;
			estadoDeMotores[3] = 0;
		}

		manejarMotores(estadoDeMotores);
		estadoMotores = estadoDeseado;

	} else {
		if (estadoMotores != estadoDeseado) {
			estadoMotores = NNNN;
		}
	}

}

void manejarMotores(int estadoDeMotores[]) {

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, estadoDeMotores[MOTOR_Y_NEGATIVO]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, estadoDeMotores[MOTOR_X_NEGATIVO]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, estadoDeMotores[MOTOR_Y_POSITIVO]);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, estadoDeMotores[MOTOR_X_POSITIVO]);
}

enum HORIZONTALIDAD_ESTADOS getEstadoDeseado() {

	int estado = 0;
	enum HORIZONTALIDAD_ESTADOS estadoATransitar;

	for (int i = 0; i < 4; i++) {
		estado = 10 * estado + estadoDeseadoMotores[i];
	}

	switch (estado) {

	case 0:
		estadoATransitar = NNNN;
		break;
	case 10:
		estadoATransitar = NNPN;
		break;
	case 11:
		estadoATransitar = NNPP;
		break;
	case 1:
		estadoATransitar = NNNP;
		break;
	case 1001:
		estadoATransitar = PNNP;
		break;
	case 1000:
		estadoATransitar = PNNN;
		break;
	case 1100:
		estadoATransitar = PPNN;
		break;
	case 100:
		estadoATransitar = NPNN;
		break;
	case 110:
		estadoATransitar = NPPN;
		break;
	default:
		estadoATransitar = NNNN;

	}
	return estadoATransitar;
}

void estabilizar(void *argument) {

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		estabilizarMotores(MOTOR_X_NEGATIVO, MOTOR_X_POSITIVO,
				Calculate_RotationX());
		estabilizarMotores(MOTOR_Y_NEGATIVO, MOTOR_Y_POSITIVO,
				Calculate_RotationY());

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

		contadorCorrectorEstabilidad++;

	} else if (rotacion > 10) {
		estadoMotorA = 1;
		estadoMotorB = 0;

		contadorCorrectorEstabilidad++;
	} else {
		estadoMotorA = 0;
		estadoMotorB = 0;
	}

	estadoDeseadoMotores[motorA] = estadoMotorA;
	estadoDeseadoMotores[motorB] = estadoMotorB;

}

void ajustarAltura(void *argument) {
	TickType_t xLastWakeTime;

	while (1) {
		xSemaphoreTake(xSemaphore, portMAX_DELAY);

		encenderOApagarMotores();

		xSemaphoreGive(xSemaphore);
		vTaskDelayUntil(&xLastWakeTime,
				pdMS_TO_TICKS(ALTITUD_TAREA_PERIODO_MS));

	}
}

void encenderOApagarMotores() {
	int alturaActual = Calculate_Hight();
	if ((alturaReferencia - alturaActual) >= 5) {
		encenderMotores = 1;
		contadorCorrectorAltitud++;
	} else {
		encenderMotores = 0;
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
