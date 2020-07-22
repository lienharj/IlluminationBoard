
#include <SPI.h>

#include <variant.h>

const int CONV = 15;      //DIGITAL OUT
const int PWR_EN = 16;    //DIGITAL OUT
const int POT_IN = 9;     //ANALOG IN
const int SWITCH_IN = 25; //DIGITAL IN
const int BUTTON = 13;    //DIGITAL IN
const int OPENLED = 12;   //DIGITAL IN
const int SIG_EN = 20;    //DIGITAL OUT
const int PWM_OUT = 19;   //PWM OUT
const int INT_TRIG = 14;  //DIGITAL OUT

uint16_t PotValue = 0;

void setup() {
  // put your setup code here, to run once:
  SPIClass SPI1 (&PERIPH_SPI, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);
  
  SPI1.begin();

  Serial1.begin(9600);

  pinMode(CONV, OUTPUT);
  pinMode(PWR_EN, OUTPUT);
  pinMode(SWITCH_IN, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(OPENLED, INPUT);
  pinMode(SIG_EN, OUTPUT);
  pinMode(INT_TRIG, OUTPUT);

  digitalWrite(CONV, HIGH);   //put SDO of LTC2312-12 in HiZ

  digitalWrite(PWR_EN, HIGH); //turn on power on LEDPCB

  if(digitalRead(SWITCH_IN))
    digitalWrite(SIG_EN, LOW);  //use external trigger if switch is high
  else
    digitalWrite(SIG_EN, HIGH); //use internal trigger if switch is low
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if(digitalRead(BUTTON))   //button signal is now trigger signal
    digitalWrite(INT_TRIG, HIGH);
  else
    digitalWrite(INT_TRIG, LOW);
  
  PotValue = analogRead(POT_IN);    //get pot value

  analogWrite(PWM_OUT, map(PotValue, 0, 1023, 0, 255));  //dimm LED according to pot value
}

unsigned int measureVoltage() {

  unsigned int measurement = 0;
  
  digitalWrite(CONV, LOW);
  delay(10);
  digitalWrite(CONV, HIGH);
  delay(10);

  digitalWrite(CONV, LOW);
  SPI1.beginTransaction(SPISettings(140000, MSBFIRST, SPI_MODE1));

  measurement = SPI1.transfer(0x00);

  measurement = measurement << 8;

  measurement |= SPI1.transfer(0x00);

  digitalWrite(CONV, HIGH);

  return measurement;
}
