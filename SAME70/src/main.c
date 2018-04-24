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

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	

	/* Configura Leds */
	LED_init(1);

	/** Configura timer TC0, canal 1 */
	//TC_init(TC0, ID_TC0, 0, 1.25);
	
	pmc_enable_periph_clk(ID_RTT);
	//pmc_enable_periph_clk(ID_RTC);
	rtt_init(RTT, 0x8000);
	rtt_disable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
	rtt_write_alarm_time(RTT, 2);
	//rtt_sel_source(RTT, true);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	NVIC_EnableIRQ((IRQn_Type) ID_RTT);
	rtt_enable(RTT);
	
	//printf("CPU: %d", sysclk_get_cpu_hz());
	//printf("SYS: %d", sysclk_get_peripheral_hz());


	while (1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		//pmc_sleep(SAM_PM_SMODE_BACKUP);
		/* Entrar em modo sleep */

	//	pin_toggle(LED_PIO, LED_PIN_MASK);
	//pio_clear(LED_PIO, LED_PIN_MASK);
		//delay_ms(500);
	}

}

