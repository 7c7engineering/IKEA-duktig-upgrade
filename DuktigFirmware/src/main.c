#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>


const uint8_t pin_buzzer_bm = PIN1_bm;
const uint8_t pin_led_1_bm = PIN2_bm;
const uint8_t pin_led_2_bm = PIN3_bm;
const uint8_t pin_but_1_bm = PIN6_bm;
const uint8_t pin_but_2_bm = PIN7_bm;

uint8_t prev_button_state = 0;
uint8_t led1_state = 0;
uint8_t led2_state = 0;
uint16_t btn1_timeout = 0;
uint16_t btn2_timeout = 0;

void init_timers(void){
  while (RTC.STATUS > 0) { /* Wait for all register to be synchronized */
	}
  // RTC will count at 1Hz
  RTC.CTRLA = RTC_PRESCALER_DIV32768_gc /* 32768 */
	            | 0 << RTC_RTCEN_bp       /* Enable: disabled */
	            | 0 << RTC_RUNSTDBY_bp;   /* Run In Standby: disabled */
  
  // setup LED PWM timer
  TCA0.SINGLE.CTRLB = 0 << TCA_SINGLE_ALUPD_bp            /* Auto Lock Update: disabled */
	                    | 0 << TCA_SINGLE_CMP0EN_bp         /* Setting: disabled */
	                    | 1 << TCA_SINGLE_CMP1EN_bp         /* Setting: enabled */
	                    | 1 << TCA_SINGLE_CMP2EN_bp         /* Setting: enabled */
	                    | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; /*  */
  
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc /* System Clock */
	                    | 1 << TCA_SINGLE_ENABLE_bp /* Module Enable: enabled */;

  // setup buzzer PWM timer (interrupt mode)
  TCB0.CTRLB = 0 << TCB_ASYNC_bp         /* Asynchronous Enable: disabled */
	             | 0 << TCB_CCMPEN_bp      /* Pin Output Enable: disabled */
	             | 0 << TCB_CCMPINIT_bp    /* Pin Initial State: disabled */
	             | TCB_CNTMODE_TIMEOUT_gc; /* Periodic Timeout */
  TCB0.INTCTRL = 1 << TCB_CAPT_bp /* Capture or Timeout: enabled */;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc  /* CLK_PER/2 (From Prescaler) */
	             | 0 << TCB_ENABLE_bp   /* Enable: disabled */
	             | 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
	             | 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */
}

void beep(uint16_t freq, uint16_t duration_ms)
{ 
  TCB0.CCMP = freq; // Find a way to calculate the correct value
  TCB0.CNT = 0;
  TCB0.CTRLA |= 1 << TCB_ENABLE_bp;
  _delay_ms(duration_ms);
  TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);
}

int main(void)
{
  ccp_write_io((void *)&(CLKCTRL.MCLKCTRLB), CLKCTRL_PDIV_2X_gc | 1 << CLKCTRL_PEN_bp);
  // take over the PSEN (also connected to the buzzer)
  PORTA.DIRSET = pin_buzzer_bm | pin_led_1_bm | pin_led_2_bm;
  PORTA.OUTSET = pin_buzzer_bm; 
  uint8_t button_state = PORTA.IN; // read the button state immediately after power on
  init_timers();
  sei();
  beep(1000, 50);
  while (1)
  {
    uint8_t new_button_state = PORTA.IN & (pin_but_1_bm | pin_but_2_bm);
    // pressing a button will change toggle the led on or off.
    // holding both buttons for 2 seconds will go to special mode
    if (new_button_state != prev_button_state)  // button state changed
    { 
      beep(1000, 50);
      if (new_button_state == pin_but_1_bm && RTC.CNT > btn1_timeout) // button 1 pressed
      {
        btn1_timeout = RTC.CNT + 1;               // set timeout to 1 seconds
        led1_state = !led1_state;                 // toggle led state
        if(led1_state) TCA0.SINGLE.CMP1 = 0xFFFF; // turn on led ()
        else TCA0.SINGLE.CMP1 = 0x0000;           // turn off led
      }
      if(new_button_state == pin_but_2_bm && RTC.CNT > btn2_timeout)        // button 2 pressed
      {
        btn2_timeout = RTC.CNT + 1;               // set timeout to 1 seconds
        led2_state = !led2_state;                 // toggle led state
        if(led2_state) TCA0.SINGLE.CMP2 = 0xFFFF; // turn on led ()
        else TCA0.SINGLE.CMP2 = 0x0000;           // turn off led
      }
    }
  }
}

ISR(TCB0_INT_vect)
{
	TCB0.INTFLAGS = TCB_CAPT_bm;
  PORTA.OUTTGL = pin_buzzer_bm;
}
    