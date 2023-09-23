#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>


const uint8_t pin_buzzer_bm = PIN1_bm;
const uint8_t pin_led_1_bm = PIN2_bm;
const uint8_t pin_led_2_bm = PIN3_bm;
const uint8_t pin_but_1_bm = PIN6_bm;
const uint8_t pin_but_2_bm = PIN7_bm;

uint8_t prev_button_state = 0;
uint8_t led1_state_index = 0;
uint8_t led2_state_index = 0;
uint16_t btn1_timeout = 0;
uint16_t btn2_timeout = 0;

#define num_brightness_levels 4
const uint16_t brightness_levels[num_brightness_levels] = {0x0000, 0x002F, 0x00AF, 0x1FFF};
const uint16_t beep_freqs[num_brightness_levels] = {11000, 8000, 6000, 4000};


void init_timers(void){
  while (RTC.STATUS > 0) { /* Wait for all register to be synchronized */
	}
  // RTC will count at 1kHz
  RTC.CTRLA = RTC_PRESCALER_DIV1024_gc /* 32768 */
	            | 1 << RTC_RTCEN_bp       /* Enable: disabled */
	            | 0 << RTC_RUNSTDBY_bp;   /* Run In Standby: disabled */
  
  // setup LED PWM timer
  TCA0.SINGLE.CTRLB = 0 << TCA_SINGLE_ALUPD_bp            /* Auto Lock Update: disabled */
	                    | 1 << TCA_SINGLE_CMP0EN_bp         /* Setting: disabled */
	                    | 0 << TCA_SINGLE_CMP1EN_bp         /* Setting: enabled */
	                    | 1 << TCA_SINGLE_CMP2EN_bp         /* Setting: enabled */
	                    | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; /*  */
  TCA0.SINGLE.PER = 0x1FF;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc /* System Clock */
	                    | 1 << TCA_SINGLE_ENABLE_bp /* Module Enable: enabled */;

  // setup buzzer PWM timer (interrupt mode)
  TCB0.INTCTRL = 1 << TCB_CAPT_bp /* Capture or Timeout: enabled */;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc  /* CLK_PER/2 (From Prescaler) */
	             | 0 << TCB_ENABLE_bp   /* Enable: disabled */
	             | 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
	             | 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */
  TCB0.CCMP = 0; // set the compare value to 0xA000
}

void beep(uint16_t freq, uint16_t duration_ms)
{ 
  TCB0.CCMP = freq; // Find a way to calculate the correct value
  TCB0.CNT = 0;
  TCB0.CTRLA |= 1 << TCB_ENABLE_bp;
  _delay_ms(duration_ms);
  TCB0.CTRLA &= ~(1 << TCB_ENABLE_bp);
  PORTA.OUTSET = pin_buzzer_bm; 
}

int main(void)
{
  ccp_write_io((void *)&(CLKCTRL.MCLKCTRLB), CLKCTRL_PDIV_2X_gc | 0 << CLKCTRL_PEN_bp); // Full chooch
  // take over the PSEN (also connected to the buzzer)
  PORTA.DIRSET = pin_buzzer_bm | pin_led_1_bm | pin_led_2_bm;
  PORTA.OUTSET = pin_buzzer_bm; 
  init_timers();
  //enable interrupts
  sei();
  TCA0.SINGLE.CMP0 = 0;
  TCA0.SINGLE.CMP2 = 0;

  while (1)
  {
    uint8_t new_button_state = PORTA.IN & (pin_but_1_bm | pin_but_2_bm);
    // pressing a button will change toggle the led on or off.
    if (new_button_state != prev_button_state)  // button state changed
    { 
      if (new_button_state == pin_but_1_bm && RTC.CNT > btn1_timeout) // button 1 pressed
      {
        led1_state_index = (led1_state_index + 1) % num_brightness_levels; // toggle led state
        TCA0.SINGLE.CMP0 = brightness_levels[led1_state_index];
        beep(beep_freqs[led1_state_index], 75);
        btn1_timeout = RTC.CNT + 2;               // set timeout to 200ms
      }
      if(new_button_state == pin_but_2_bm && RTC.CNT > btn2_timeout)        // button 2 pressed
      {
        led2_state_index = (led2_state_index + 1) % num_brightness_levels; // toggle led state
        TCA0.SINGLE.CMP2 = brightness_levels[led2_state_index];
        beep(beep_freqs[led2_state_index], 75);
        btn2_timeout = RTC.CNT + 2;               // set timeout to 200ms
        
      }
    }
    if(RTC.CNT > 1000){
      beep(16000, 75);
      PORTA.OUTCLR = pin_buzzer_bm; 
      _delay_ms(100);
    }
    prev_button_state = new_button_state;
  }
}

ISR(TCB0_INT_vect)
{
	TCB0.INTFLAGS = TCB_CAPT_bm;
  PORTA.OUTTGL = pin_buzzer_bm;
}
    