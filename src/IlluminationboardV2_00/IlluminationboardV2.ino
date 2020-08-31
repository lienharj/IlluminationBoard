/*************************************MAIN HEADER*************************************
 * 
 * @project:  Illuminationboard V2
 * @author:   Laurin Heitzer, Michael Riner
 * 
 * @brief:    software for Illuminationboard V2 Rev1, reads potentiometer and outputs
 *            pwm according to it.
 *            
 * @version:  1.0
 * 
 *************************************************************************************/
#define USE_USBCON

#include "Arduino.h"

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <SPI.h>

#define PWR_EN      3
#define INT_TRIG   12
#define PWM_MCU     4
#define EXT_SIG_EN  5
#define LED_SWITCH  8
#define BRIGHTNESS A3
#define RES_ADC    8
#define FILTER     100

#define PWM_PER 255

bool ros_used = false;
bool ros_trigger_used = false;

void init_clock_gclk5();
void init_timer_tcc0();
void init_pwm();
void pwm_set(uint8_t dutycycle);
void pwm_set_rosserial(const std_msgs::UInt8& msg);
void trigger_set_rosserial(const std_msgs::Bool& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt8> pwm_sub("Illuminationboard/pwm", pwm_set_rosserial);
ros::Subscriber<std_msgs::Bool> trigger_sub("Illuminationboard/trigger", trigger_set_rosserial);

void setup() {

  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(pwm_sub);
  nh.subscribe(trigger_sub);

  pinMode(PWR_EN, OUTPUT);
  pinMode(INT_TRIG, OUTPUT);
  pinMode(PWM_MCU, OUTPUT);
  pinMode(EXT_SIG_EN, OUTPUT);
  pinMode(LED_SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(LED_SWITCH), ISR_Switch, CHANGE);
  digitalWrite(INT_TRIG, digitalRead(LED_SWITCH));
  
  init_clock_gclk5();
  init_timer_tcc0();
  init_pwm();

  SPI.begin();             // initialize SPI for temperature sensor (ADC)


//  nh.initNode();
//  nh.subscribe(sub);
//  Serial1.begin(9600);      // hardware serial
//  Serial.begin(9600);       // USB serial

  digitalWrite(PWR_EN, 1);  // enables power on the LEDPCB
}

void ISR_Switch() {
    delay(100);
    digitalWrite(INT_TRIG, digitalRead(LED_SWITCH));
}

  static int avg[FILTER];
  static int i = 0;
  int mv_avg = 0;
  
void loop() {

	nh.spinOnce();

  if(!ros_used)
  {
    analogReadResolution(RES_ADC);
    analogWriteResolution(RES_ADC);
    avg[i] = analogRead(BRIGHTNESS);
    
    if (i < FILTER)
        i++;
    else
        i = 0;
    
    for (int j = 0; j < FILTER; j++)
        mv_avg += avg[j];
    mv_avg = mv_avg / FILTER;
    
    if (mv_avg > 250)
        mv_avg = 255;

    digitalWrite(EXT_SIG_EN, true); 
      
    // write ADC average into ccb0 register of tcc0 to set dutycycle of PWM
    pwm_set(mv_avg);
  }
}

/**
 * @function  init_clock_gclk5
 * @author    Laurin Heitzer
 * 
 * @brief     initializes GCLK5 with 8MHz internal oscillator as source
 * 
*/
void init_clock_gclk5()
{
    // Set the divisor for GCLK5.
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  // Set divisor to 1
                    GCLK_GENDIV_ID(5);    // For GCLK5  
  while(GCLK->STATUS.bit.SYNCBUSY); 
  
    // Set the clock source, duty cycle, and enable GCLK5  
  REG_GCLK_GENCTRL = GCLK_GENCTRL_SRC_OSC8M |    // Set 8MHz source
                     GCLK_GENCTRL_IDC |          // Improve Duty Cycle
                     GCLK_GENCTRL_GENEN |        // Enable GCLK
                     GCLK_GENCTRL_ID(5);         // For GLCK5    
  while(GCLK->STATUS.bit.SYNCBUSY);    
}

/**
 * @function  init_timer_tcc0
 * @author    Laurin Heitzer
 * 
 * @brief     initializes timer TCC0 with GCLK5 as clock source
 * 
*/
void init_timer_tcc0()
{
    // Route GLCK5 to TCC0 & TCC1, and enable the clock.
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC0_TCC1 | // Route GCLK5 to TCC0 & 1
                     GCLK_CLKCTRL_CLKEN |        // Enable the clock
                     GCLK_CLKCTRL_GEN_GCLK5;     // Select GCLK5
  while(GCLK->STATUS.bit.SYNCBUSY);  
}

/**
 * @function  init_pwm
 * @author    Laurin Heitzer
 * 
 * @brief     initializes PWM and routes it to PB10. PWM frequency is ~31.3kHz
 * 
*/
void init_pwm()
{
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  REG_TCC0_PER = PWM_PER;
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1
             | TCC_CTRLA_ENABLE;     // Requires SYNC on CTRLA
  while( TCC0->SYNCBUSY.bit.ENABLE
             || TCC0->SYNCBUSY.bit.WAVE
             || TCC0->SYNCBUSY.bit.PER );  
             
  PORT->Group[g_APinDescription[4].ulPort]
    .PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
  
  PORT->Group[g_APinDescription[4].ulPort]
    .PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
  
  REG_TCC0_CCB0 = 0;    //set dutycycle to 0 at start (LED off)
  while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  pwm_set
 * @author    Laurin Heitzer
 * 
 * @param			uint8_t dutycycle
 * @brief     sets the pwm dutycycle (range: 0-100)
 * 
*/
void pwm_set(uint8_t dutycycle)
{
  REG_TCC0_CCB0 = dutycycle;
    while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  pwm_set_rosserial
 * @author    Laurin Heitzer
 * 
 * @param			std_msgs::UInt8 msg
 * @brief     sets the pwm dutycycle via rosserial (range: 0-100) and disables adc pot pwm
 * 
*/
void pwm_set_rosserial(const std_msgs::UInt8& msg)
{
  ros_used = true;

  REG_TCC0_CCB0 = msg.data; //sets pwm dutycycle (range: 0-100)
    while(TCC0->SYNCBUSY.bit.CCB0);
}

/**
 * @function  pwm_set_rosserial
 * @author    Laurin Heitzer
 * 
 * @param			std_msgs::Bool msg
 * @brief     disables the trigger switch and enables ros trigger
 * 
*/
void trigger_set_rosserial(const std_msgs::Bool& msg)
{
  if(!ros_trigger_used)
  {
    detachInterrupt(digitalPinToInterrupt(LED_SWITCH));
    ros_trigger_used = true;
  }

  if(msg.data == true)
  {
    digitalWrite(INT_TRIG, HIGH);
  }
  else if(msg.data == false)
  {
    digitalWrite(INT_TRIG, LOW);
  }
}
