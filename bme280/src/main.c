/**
 *  Computação Embarcada
 *  Insper
 *  Rafael Corsi - rafael.corsi@insper.edu.br
 *  Código exemplo uso I2C
 *  Maio - 2019
 *
 *  Bug conhecido : spi read deve ser executado duas vezes ?
 *      - funcao afetada : mcu6050_i2c_bus_read()
 *
 *  Conectar :
 *            MCU  |  SAME70-XPLD | 
 *           ----------------------
 *            SDA  |   EXT2-11    |  PA3
 *            SCL  |   EXT2-12    |  PA4
 *            GND  |   EXT2-19    |
 *            VCC  |   EXT2-20    | 
 * 
 * TODO: calibracao
 *       bug dois reads i2c
 */



#include <asf.h>
#include "conf_board.h"
#include <string.h>
#include "bme280.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define YEAR        2019
#define MOUNTH      1
#define DAY         1
#define WEEK        1
#define HOUR        13
#define MINUTE      43
#define SECOND      34



/**
 * LEDs
 */
#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID            ID_PIOA
#define BUT_PIO               PIOA
#define BUT_PIN		            11
#define BUT_PIN_MASK          (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

//butao 1 oled
#define EBUT1_PIO PIOD //start EXT 9 PD28
#define EBUT1_PIO_ID 16
#define EBUT1_PIO_IDX 28
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)
//butao 2 oled
#define EBUT2_PIO PIOA //pause  Ext 4 PA19 PA = 10
#define EBUT2_PIO_ID 10
#define EBUT2_PIO_IDX 19
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)
//butao 3 oled
#define EBUT3_PIO PIOC //sei la EXT 3 PC31
#define EBUT3_PIO_ID 12 // piod ID
#define EBUT3_PIO_IDX 31
#define EBUT3_PIO_IDX_MASK (1u << EBUT3_PIO_IDX)

#define TWIHS_MCU6050_ID    ID_TWIHS0
#define TWIHS_MCU6050       TWIHS0

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;

int16_t  accX, accY, accZ;
volatile uint8_t  accXHigh, accYHigh, accZHigh;
volatile uint8_t  accXLow,  accYLow,  accZLow;
uint32_t hour, minuto,seg;
uint temp, pres, umid;
int opc = 1;
int delay = 1000/portTICK_PERIOD_MS;






/** RTOS  */

#define TASK_SD_STACK_SIZE            (200*1024/sizeof(portSTACK_TYPE))
#define TASK_SD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_USUARIO_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_USUARIO_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_PROCESS_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (tskIDLE_PRIORITY)

QueueHandle_t xQueueBot1;
QueueHandle_t xQueueBot2;
SemaphoreHandle_t xSemaphore;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void pin_toggle(Pio *pio, uint32_t mask);


/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
  pin_toggle(PIOD, (1<<28));
  pin_toggle(LED_PIO, LED_PIN_MASK);
}

static void but1_oled_callback(void){
	opc++;
}
static void but2_oled_callback(void){
	if (delay > 100)
	delay -= 100;
}
static void but3_oled_callback(void){
	delay += 100;
}


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 5);
	
		// configura botoes do oled
		pmc_enable_periph_clk(EBUT1_PIO_ID);
		pmc_enable_periph_clk(EBUT2_PIO_ID);
		pmc_enable_periph_clk(EBUT3_PIO_ID);
		
		// configura botoes do oled como input;
		pio_configure(EBUT1_PIO, PIO_INPUT, EBUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_configure(EBUT2_PIO, PIO_INPUT, EBUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_configure(EBUT3_PIO, PIO_INPUT, EBUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		
		// Configura interrup??o no pino referente ao botao e associa
		// fun??o de callback caso uma interrup??o for gerada
		// a fun??o de callback ? a: but_callback()
		pio_handler_set(EBUT1_PIO,
		EBUT1_PIO_ID,
		EBUT1_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but1_oled_callback);
		
		pio_handler_set(EBUT2_PIO,
		EBUT2_PIO_ID,
		EBUT2_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but2_oled_callback);
		
		pio_handler_set(EBUT3_PIO,
		EBUT3_PIO_ID,
		EBUT3_PIO_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but3_oled_callback);
		
		// Ativa interrup??o
		pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT3_PIO, EBUT3_PIO_IDX_MASK);

		// Configura NVIC para receber interrupcoes do PIO do botao
		// com prioridade 4 (quanto mais pr?ximo de 0 maior)

		NVIC_EnableIRQ(EBUT1_PIO_ID);
		NVIC_SetPriority(EBUT1_PIO_ID, 4); // Prioridade 4
		NVIC_EnableIRQ(EBUT2_PIO_ID);
		NVIC_SetPriority(EBUT2_PIO_ID, 4); // Prioridade 4
		NVIC_EnableIRQ(EBUT3_PIO_ID);
		NVIC_SetPriority(EBUT3_PIO_ID, 4); // Prioridade 4
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};




static void configure_console(void){
	  /* Configura USART1 Pinos */
	  sysclk_enable_peripheral_clock(ID_PIOB);
	  sysclk_enable_peripheral_clock(ID_PIOA);
	  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
		
		
		const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);

}


/************************************************************************/
/* BME                                                                  */
/************************************************************************/ 
/*	
 *  \Brief: The function is used as I2C bus init
 */
void bme280_i2c_bus_init(void)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 10000;
	twihs_master_init(TWIHS_MCU6050, &bno055_option);
}

uint8_t bme280_i2c_read_reg(uint CHIP_ADDRESS, uint reg_address, char *value){
  	uint i = 1;
    
  	  twihs_packet_t p_packet;
  	  p_packet.chip         = CHIP_ADDRESS;//BME280_ADDRESS;
  	  p_packet.addr_length  = 0;

  	  char data = reg_address; //BME280_CHIP_ID_REG;
  	  p_packet.buffer       = &data;
  	  p_packet.length       = 1;
  	
  	  if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	    return 1;

  	  p_packet.addr_length  = 0;
  	  p_packet.length       = 1;
      p_packet.buffer       = value;

  	  if(twihs_master_read(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	    return 1;
        
    return 0;  
}

int8_t bme280_i2c_config_temp(void){
  	int32_t ierror = 0x00;
  	
  	twihs_packet_t p_packet;
  	p_packet.chip         = BME280_ADDRESS;//BME280_ADDRESS;
    p_packet.addr[0]      = BME280_CTRL_MEAS_REG;
  	p_packet.addr_length  = 1;

  	char data = 0b00100111; //BME280_CHIP_ID_REG;
  	p_packet.buffer       = &data;
  	p_packet.length       = 1;
  	
  	if(twihs_master_write(TWIHS_MCU6050, &p_packet) != TWIHS_SUCCESS)
  	return 1;
}


int8_t bme280_i2c_read_temp(uint *temp, uint *pres, uint *umid)
{
  int32_t ierror = 0x00;
  char tmp[3];
  char pr[3];
  char um[3];
  
  
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_MSB_REG, &tmp[2]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_TEMPERATURE_LSB_REG, &tmp[1]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &pr[2]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_MSB_REG, &pr[2]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &pr[1]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_PRESSURE_LSB_REG, &pr[1]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &um[2]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_MSB_REG, &um[2]);
  
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &um[1]);
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_HUMIDITY_LSB_REG, &um[1]);  
  
 
  
  *temp = tmp[2] << 8 | tmp[1];
  *umid = um[2] << 8 | um[1];
  *pres = pr[2] << 8 | pr[1];
	return 0;
}

uint8_t bme280_validate_id(void){
  char id;
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id );
  bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id );
  if (bme280_i2c_read_reg(BME280_ADDRESS, BME280_CHIP_ID_REG, &id )) 
    return 1;
  if (id != 0x60)
    return 1;
  return 0; 
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}



  /* buffer para recebimento de dados */
  uint8_t bufferRX[100];
  uint8_t bufferTX[100];
  
  uint8_t rtn;
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_sensor(void){
  
 while(1){
	 if (bme280_i2c_read_temp(&temp, &pres, &umid))
	 printf("erro readinG temperature \n");
	 
	 pin_toggle(LED_PIO, LED_PIN_MASK);
	 vTaskDelay(delay);
 }
}

	
void task_usuario(void){
	
	char hnum1[5];
	char hnum2[5];
	char hnum3[5];
	RTC_init();
	BUT_init();
	gfx_mono_ssd1306_init();  

	while(1){
		rtc_get_time(RTC, &hour, &minuto, &seg);
	
	if (opc % 2 == 0) {
		printf("RUN MODE, Aperte o botão 1 para gravar.\n");
		printf("%d:%d:%d\n", hour, minuto, seg);
		printf("Temperatura: %d \n", temp);
		printf("Pressao: %d \n", pres);
		printf("Umidade: %d \n\n", umid);
		
		//Dividindo por 1000 para caber na tela OLED
		itoa(temp/1000, hnum1, 10);
		itoa(umid/1000, hnum2, 10);
		itoa(pres/1000, hnum3, 10);
		gfx_mono_draw_string("       ",0,16, &sysfont);
		gfx_mono_draw_string("         ",45,16, &sysfont);
		gfx_mono_draw_string("T:",0,16, &sysfont);
		gfx_mono_draw_string(hnum1,20,16, &sysfont);
		gfx_mono_draw_string("U:",45,16, &sysfont);
		gfx_mono_draw_string(hnum2,65,16, &sysfont);
		gfx_mono_draw_string("P:",90,16, &sysfont);
		gfx_mono_draw_string(hnum3,110,16, &sysfont);
	} else {
		gfx_mono_draw_string("       ",0,16, &sysfont);
		gfx_mono_draw_string("         ",45,16, &sysfont);
		gfx_mono_draw_string("Record mode",0,16, &sysfont);
		xSemaphoreGive(xSemaphore);
	}
	
		
	vTaskDelay(delay);
	
	}
}
	
void task_sd(void){
	
		xSemaphore = xSemaphoreCreateBinary();	
		
		char hnum1[5];
		char hnum2[5];
		char hnum3[5];
		char hnum4[5];
		char hnum5[5];
		char hnum6[5];
		
		char test_file_name[] = "0:sd_mmc_test.txt";
		Ctrl_status status;
		FRESULT res;
		FATFS fs;
		FIL file_object;
		
		irq_initialize_vectors();
		cpu_irq_enable();
		
		/* Initialize SD MMC stack */
		sd_mmc_init();
		
		//printf("\x0C\n\r-- SD/MMC/SDIO Card Example on FatFs --\n\r");
		//printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	while(1){
		if( xSemaphoreTake(xSemaphore, ( TickType_t ) 10) == pdTRUE){
		printf("Please plug an SD, MMC or SDIO card in slot.\n\r");

		itoa(temp, hnum1, 10);
		itoa(umid, hnum2, 10);
		itoa(pres, hnum3, 10);
		itoa(hour, hnum4, 10);
		itoa(minuto, hnum5, 10);
		itoa(seg, hnum6, 10);	
		
		
		/* Wait card present and ready */
		do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("Card install FAIL\n\r");
				printf("Please unplug and re-plug the card.\n\r");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
		} while (CTRL_GOOD != status);

		printf("Mount disk (f_mount)...\r\n");
		memset(&fs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
		if (FR_INVALID_DRIVE == res) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");

		printf("Create a file (f_open)...\r\n");
		test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
		(char const *)test_file_name,
		FA_CREATE_ALWAYS | FA_WRITE);
		if (res != FR_OK) {
			printf("[FAIL] res %d\r\n", res);
			goto main_end_of_test;
		}
		printf("[OK]\r\n");

		printf("Write to test file (f_puts)...\r\n");
		if (0 == f_puts("Temperatura-", &file_object)) {
			f_close(&file_object);
			printf("[FAIL]\r\n");
			goto main_end_of_test;
		}
		f_puts(hnum1, &file_object);
		f_puts("  Pressão-", &file_object);
		f_puts(hnum3, &file_object);
		f_puts("  Umidade-", &file_object);
		f_puts(hnum2, &file_object);
		f_puts(" ", &file_object);
		f_puts(hnum4, &file_object);
		f_puts(":", &file_object);
		f_puts(hnum5, &file_object);
		f_puts(":", &file_object);
		f_puts(hnum6, &file_object);
		printf("[OK]\r\n");
		f_close(&file_object);
		printf("Test is successful.\n\r");

		main_end_of_test:
		printf("Please unplug the card.\n\r\n");
		while (CTRL_NO_PRESENT != sd_mmc_check(0)) {}		
			
		}
		}
	}
		

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void){
	  /* Initialize the SAM system */
  sysclk_init();
  board_init();
   
  /* Disable the watchdog */
  WDT->WDT_MR = WDT_MR_WDDIS;
  
  /* Inicializa com serial com PC*/
  configure_console();
  printf("Demo do sensor BME280, sem calibracao! \n");
  
  /* Configura Leds */
  LED_init(1);
  
  /* Inicializa funcao de delay */
  delay_init( sysclk_get_cpu_hz());
   
  
  /************************************************************************/
  /* MPU                                                                  */
  /************************************************************************/
  
  /* Inicializa i2c */
  printf("Inicializando bus i2c \n");
  bme280_i2c_bus_init();
  delay_ms(10);
  
  /* verificando presenca do chip */
  while(bme280_validate_id()){
    printf("Chip nao encontrado\n");
   delay_ms(200);
  }    
   
  printf("Chip encontrado, inicializando temperatura \n");
  bme280_i2c_config_temp();
	
	
	/* Create task to make led blink */
	xTaskCreate(task_sensor, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
  
	if (xTaskCreate(task_sd, "SD", TASK_SD_STACK_SIZE, NULL, TASK_SD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create SD task\r\n");
	}
	
	if (xTaskCreate(task_usuario, "usuario", TASK_USUARIO_STACK_SIZE, NULL, TASK_USUARIO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create USU task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){

 }

	

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
