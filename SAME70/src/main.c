#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/**
* LEDs
*/
#define LED_PIO_ID	   ID_PIOD
#define LED_PIO        PIOD
#define LED_PIN		   30
#define LED_PIN_MASK   (1<<LED_PIN)

#define USART_COM_ID ID_USART1
#define USART_COM    USART1


static void USART1_init(void);

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	
	
	pin_toggle(LED_PIO, LED_PIN_MASK);		
}

/************************************************************************/
/* Funcoes                                                              */
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
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS); //INTERRUPCAO

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}


void RTT_Handler(void) {
	 rtt_get_status(RTT);
	 rtt_write_alarm_time(RTT, rtt_read_timer_value(RTT) + 1);
	 //volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	//ul_dummy = rtt_get_status(RTT);

	/* Avoid compiler warning */
	//UNUSED(ul_dummy);

	/** Muda o estado do LED */
	
	pin_toggle(LED_PIO, LED_PIN_MASK);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};


	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}


static void USART1_init(void){
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate       = 115200,
		.char_length    = US_MR_CHRL_8_BIT,
		.parity_type    = US_MR_PAR_NO,
		.stop_bits   	= US_MR_NBSTOP_1_BIT	,
		.channel_mode   = US_MR_CHMODE_NORMAL
	};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(USART_COM_ID);

	/* Configura USART para operar em modo RS232 */
	usart_init_rs232(USART_COM, &usart_settings, sysclk_get_peripheral_hz());

	/* Enable the receiver and transmitter. */
	usart_enable_tx(USART_COM);
	usart_enable_rx(USART_COM);

	/* map printf to usart */
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

	/* ativando interrupcao */
	usart_enable_interrupt(USART_COM, US_IER_RXRDY);
	NVIC_SetPriority(USART_COM_ID, 4);
	NVIC_EnableIRQ(USART_COM_ID);

}


void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART_COM);

	char c;

	// Verifica por qual motivo entrou na interrupçcao
	//  - Dadodisponível para leitura
	if(ret & US_IER_RXRDY){
		usart_serial_getchar(USART_COM, &c);

		// -  Transmissoa finalizada
		} else if(ret & US_IER_TXRDY){
			return;

	}
}
/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/* Initialize the console uart */
	configure_console();
	USART1_init();
	/* Configura Leds */
	LED_init(0);

	/** Configura timer TC0, canal 1 */
	//TC_init(TC0, ID_TC0, 0, 1.25);
	
	pmc_enable_periph_clk(ID_RTT);
	//pmc_enable_periph_clk(ID_RTC);
	rtt_init(RTT, 0x6000);
	rtt_disable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
	rtt_write_alarm_time(RTT, 1);
	//rtt_sel_source(RTT, true);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	NVIC_EnableIRQ((IRQn_Type) ID_RTT);
	rtt_enable(RTT);
	
	//printf("CPU: %d", sysclk_get_cpu_hz());
	//printf("SYS: %d", sysclk_get_peripheral_hz());
	
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL);
	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTTEN_ENABLE);

	while (1) {
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		pmc_sleep(SAM_PM_SMODE_BACKUP);
		/* Entrar em modo sleep */

	//	pin_toggle(LED_PIO, LED_PIN_MASK);
	//pio_clear(LED_PIO, LED_PIN_MASK);
		//delay_ms(500);
	}

}

