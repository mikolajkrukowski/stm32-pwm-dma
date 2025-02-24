/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#define BUFF_SIZE 512
#define USART_TXBUF_LEN 128
#define USART_RXBUF_LEN 256

/*	Bufor odbiorczy	*/
uint8_t USART_RxBuf[USART_RXBUF_LEN];
__IO int USART_RX_Empty = 0; //wskaznik zapisu
__IO int USART_RX_Busy = 0; //wskaznik odczytu

/*	Bufor nadawczy	*/
uint8_t USART_TxBuf[USART_TXBUF_LEN];
__IO int USART_TX_Empty = 0;
__IO int USART_TX_Busy = 0;

/*	Ramka	*/
uint8_t buforRamki[USART_RXBUF_LEN];
uint8_t stan = 0;

char device_address[4]="STM";
char source_address[4]="USR";
char destination_address[4];
uint16_t dlugoscRamki = 0;
int cmdLength;
uint8_t znak;
char wiadomosc[USART_RXBUF_LEN];

//======= ZMIENNE - suma kontrolna ===========
char suma_kontrolna[6];
uint16_t suma_wyliczona = 0;
int podana_suma_kontrolna=0;

/*	Zegar	*/
uint16_t buffT1[BUFF_SIZE];
uint32_t IC_Values[2] = {0, 0};  // Bufor DMA: [0] - period, [1] - duty
uint32_t pwmPeriod = 0;          // Zmienna przechowująca okres sygnału PWM
uint32_t pwmDuty = 0;            // Zmienna przechowująca szerokość impulsu (duty cycle)
uint16_t pwmBuffer[BUFF_SIZE];
uint32_t setFrequency = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//	Liczenie sumy
uint16_t CRC16(const char *data, size_t length) {
    uint16_t crc = 0xFFFF; // Wartość początkowa
    uint16_t poly = 0xA001; // Polinom CRC16

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i]; // XOR z bieżącym bajtem

        for (int j = 0; j < 8; j++) { // 8 iteracji dla każdego bitu bajtu
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ poly; // XOR z polinomem, gdy najniższy bit jest ustawiony
            } else {
                crc >>= 1; // Przesunięcie w prawo
            }
        }
    }
    return crc; // Zwróć wynik w 16 bitach
}


//		Obsługa przerwań
/*  USART CALLABACK ODBIOR	*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		USART_RX_Empty++;
		if(USART_RX_Empty >= USART_RXBUF_LEN)
			USART_RX_Empty = 0;
		HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
	}
}

/*  USART CALLABACK NADAWAWNIE	*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
		if(USART_TX_Empty != USART_TX_Busy){
			uint8_t tmp = USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if(USART_TX_Busy >= USART_TXBUF_LEN)
				USART_TX_Busy = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}


/* Obsługa timerów*/

void HAL_DMA_TxHalfCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma == &hdma_tim1_ch1) {
        // Aktualizacja danych w pierwszej połowie bufora
        for (int i = 0; i < BUFF_SIZE / 2; i++) {
            buffT1[i] = (i % 100) + 100;
        }
    }
}

void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma == &hdma_tim1_ch1) {
        // Aktualizacja danych w drugiej połowie bufora
        for (int i = BUFF_SIZE / 2; i < BUFF_SIZE; i++) {
            buffT1[i] = (i % 100) + 200;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        pwmPeriod = IC_Values[0]; // Okres PWM
        pwmDuty = IC_Values[1];   // Szerokość impulsu PWM (wypełnienie)
    }
}



/*	USART_fsend	*/
void USART_fsend(char* format, ...){
	char tmp_rs[128];
	int i;
	__IO int idx;
	va_list arglist; //tworzenie listy argumentów
	va_start(arglist, format);
	vsprintf(tmp_rs, format, arglist); //przenoszenie danych z listy do tablicy
	va_end(arglist);
	idx = USART_TX_Empty; //przypisywanie pozycji wskaźnika do wskaźnika pomocniczego

	for(i=0; i<strlen(tmp_rs); i++){ //przenoszenie danych do bufora
		USART_TxBuf[idx] = tmp_rs[i]; // Zapisanie danych do bufora nadawczego
		idx++;
		if(idx >= USART_TXBUF_LEN) //sprawdzanie zakresu wskaźnika
			idx = 0;
	}
	__disable_irq();
	if((USART_TX_Empty == USART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)){
		USART_TX_Empty = idx; // Aktualizacja wskaźnika pustego
		uint8_t tmp = USART_TxBuf[USART_TX_Busy]; // Pobranie pierwszego bajtu do wysyłki
		USART_TX_Busy++;
		if(USART_TX_Busy >= USART_TXBUF_LEN) //// Obsługa przepełnienia wskaźnika
			USART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}else{
		USART_TX_Empty = idx;
	}
	__enable_irq();
}


/*   Analiza komendy / Obsługa rozkazów   */
void analizaKomendy(char cmd[]) {
	if (!strncmp(cmd, "SET_PWM", 7)) {
	    uint16_t newDuty = atoi(&cmd[7]);

	    if (newDuty <= 100) {  // Wypełnienie w % (0-100%)
	        uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1); // Pobranie aktualnego okresu
	        uint32_t ccr = (newDuty * (period + 1)) / 100; // Obliczenie wartości CCR

	        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr); // Ustawienie nowego wypełnienia PWM

	        uint8_t data[20];
	        snprintf((char*)data, sizeof(data), "CURRENT_PWM:%u", newDuty);
	        uint16_t checksum = CRC16(data, strlen((char*)data));
	        sprintf(wiadomosc, "!%s%s%05uCURRENT_PWM:%u;\r\n", device_address, source_address, checksum, newDuty);
	        USART_fsend(wiadomosc);
	    } else {
	        errorKomenda();
	    }

    } else if (!strncmp(cmd, "SET_FREQ", 8)) {
        uint32_t newFreq = atoi(&cmd[8]);
        if (newFreq > 0 && newFreq <= 36000000) { // Możemy ustawić do 36 MHz
            uint32_t prescaler = 0;
            uint32_t period;

            // Automatyczne dobieranie preskalera i ARR
            while ((72000000 / (prescaler + 1)) > (newFreq * 65536)) {
                prescaler++;
            }

            period = (72000000 / ((prescaler + 1) * newFreq)) - 1;
            __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
            __HAL_TIM_SET_AUTORELOAD(&htim1, period);

            setFrequency=newFreq;

            uint8_t data[20];
            snprintf((char*)data, sizeof(data), "CURRENT_FREQ:%lu", newFreq);
            uint16_t checksum = CRC16(data, strlen((char*)data));
            sprintf(wiadomosc, "!%s%s%05uCURRENT_FREQ:%lu;\r\n", device_address, source_address, checksum, newFreq);
            USART_fsend(wiadomosc);
        } else {
            errorKomenda();
        }

    } else if (!strncmp(cmd, "GET_IN_PWM", 10)) { // Pobranie aktualnego okresu i wypełnienia PWM
        uint32_t frequency;
        uint32_t duty_cycle;

        if (pwmPeriod > 0) {
            // Obliczenie częstotliwości i wypełnienia na podstawie DMA
            frequency = 72000000 / pwmPeriod;
            duty_cycle = (pwmDuty * 100) / pwmPeriod; // Pełne % wypełnienia
        } else {
            // Brak sygnału wejściowego – zwróć ostatnio ustawioną wartość częstotliwości
            frequency = setFrequency;
            duty_cycle = (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) * 100) / (__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
        }

        uint8_t data[40];
        snprintf((char*)data, sizeof(data), "FREQ:%luHz DUTY:%lu%%", frequency, duty_cycle);
        uint16_t checksum = CRC16(data, strlen((char*)data));

        sprintf(wiadomosc, "!%s%s%05uFREQ:%luHz DUTY:%lu%%;\r\n",
                device_address, source_address, checksum, frequency, duty_cycle);
        USART_fsend(wiadomosc);
    }
}



/*   Komunikaty zwrotne   */
void start() {
    uint8_t data[6 + strlen("START")]; // 3 bajty adresu źródła, 3 bajty adresu urządzenia, długość komendy
    memcpy(data, source_address, 3);
    memcpy(data + 3, device_address, 3);
    memcpy(data + 6, "START", strlen("START"));
    uint16_t crc = CRC16(data, sizeof(data));
    sprintf(wiadomosc, "!%s%s%05uSTART;\r\n", device_address, source_address, crc);
    USART_fsend(wiadomosc);
}

void odebranoRamke() {
    uint8_t data[6 + strlen("OdebranoRamke")];
    memcpy(data, source_address, 3);
    memcpy(data + 3, device_address, 3);
    memcpy(data + 6, "OdebranoRamke", strlen("OdebranoRamke"));
    uint16_t crc = CRC16(data, sizeof(data));
    sprintf(wiadomosc, "!%s%s%05uOdebranoRamke;\r\n", device_address, source_address, crc);
    USART_fsend(wiadomosc);
}

void errorKomenda() {
    uint8_t data[6 + strlen("CMDError")];
    memcpy(data, source_address, 3);
    memcpy(data + 3, device_address, 3);
    memcpy(data + 6, "CMDError", strlen("CMDError"));
    uint16_t crc = CRC16(data, sizeof(data));
    sprintf(wiadomosc, "!%s%s%05uCMDError;\r\n", device_address, source_address, crc);
    USART_fsend(wiadomosc);
}

void errorSumaKontrolna() {
    uint8_t data[6 + strlen("SUMError")];
    memcpy(data, source_address, 3);
    memcpy(data + 3, device_address, 3);
    memcpy(data + 6, "SUMError", strlen("SUMError"));
    uint16_t crc = CRC16(data, sizeof(data));
    sprintf(wiadomosc, "!%s%s%05uSUMError;\r\n", device_address, source_address, crc);
    USART_fsend(wiadomosc);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  	  for (int i = 0; i < BUFF_SIZE; i++) {
  		  pwmBuffer[i] = 500; // Wypełnienie 50%
  	  }



  	  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, &IC_Values[0], 1); // Okres PWM
  	  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, &IC_Values[1], 1); // Wypełnienie

  	  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, buffT2, 512);


  	  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1); //inicjalizacja odbierania znaku
  	  start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1) {
  	    if (USART_RX_Busy != USART_RX_Empty) {
  	        char znak = USART_RxBuf[USART_RX_Busy];
  	        USART_RX_Busy++;
  	        if (USART_RX_Busy >= USART_RXBUF_LEN) USART_RX_Busy = 0;

  	        if (znak == '!') {
  	            stan = 1;
  	            dlugoscRamki = 0;
  	        } else if (stan == 1) {
  	        	if (znak == ';') {
  	                buforRamki[dlugoscRamki] = '\0'; // Dodanie znaku końca

  	                if (dlugoscRamki >= 11) { // Minimalna długość ramki
  	                    memcpy(source_address, &buforRamki[0], 3);
  	                    memcpy(destination_address, &buforRamki[3], 3);
  	                    memcpy(suma_kontrolna, &buforRamki[6], 5);

  	                    source_address[3] = '\0';
  	                    destination_address[3] = '\0';
  	                    suma_kontrolna[5] = '\0';

  	                    // Konwersja sumy kontrolnej na liczbę
  	                    podana_suma_kontrolna = atoi(suma_kontrolna);

  	                    // Obliczenie długości komendy
  	                    cmdLength = dlugoscRamki - 11;
  	                    if (strncmp(device_address, destination_address, 3) == 0) {
  	                        char cmd[cmdLength + 1];
  	                        memcpy(cmd, &buforRamki[11], cmdLength);
  	                        cmd[cmdLength] = '\0';

  	                        // Przygotowanie danych do CRC
  	                        uint8_t data[6 + cmdLength]; // Adresy + komenda
  	                        memcpy(data, source_address, 3);
  	                        memcpy(data + 3, destination_address, 3);
  	                        memcpy(data + 6, cmd, cmdLength);

  	                        // Obliczanie sumy kontrolnej
  	                        suma_wyliczona = CRC16(data, sizeof(data));

  	                        // Weryfikacja sumy kontrolnej
  	                        if (podana_suma_kontrolna == suma_wyliczona) {
  	                            odebranoRamke();
  	                            analizaKomendy(cmd);
  	                        } else {
  	                            errorSumaKontrolna();
  	                        }
  	                        suma_wyliczona = 0; // Resetowanie sumy
  	                    }
  	                }

  	                stan = 0; // Powrót do stanu początkowego
  	                dlugoscRamki = 0;
  	            } else {
  	                buforRamki[dlugoscRamki++] = znak;
  	                if (dlugoscRamki >= 77) {
  	                    dlugoscRamki = 0;
  	                    stan = 0; //
  	                }
  	            }
  	    }
  	}
  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
