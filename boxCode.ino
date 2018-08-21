#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <TH02_dev.h>
#include <Wire.h>
#include "SoftwareI2C.h"
#include <SoftwareSerial.h>
#include "MutichannelGasSensor.h"
#include <Arduino.h>

#define GAS_EN      1

#define TH02_EN     1




const int ss =53;
const int pin_scl = 2;      // select a pin as SCL of software I2C

const int pin_sda = 3;      // select a pin as SDA of software I2C



#define MQ135_DEFAULTPPM 399 //default ppm of CO2 for calibration
#define MQ135_RO 68550 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2
#define co2PIN A5 //Analog Pin for CO2 MQ135
#define PIN_GATE_IN 2
#define IRQ_GATE_IN  0
#define PIN_LED_OUT 13
#define PIN_ANALOG_IN A0

void soundISR()
{
  int pin_val;

  pin_val = digitalRead(PIN_GATE_IN);
  digitalWrite(PIN_LED_OUT, pin_val);   
}
double mq135_getppm(long resvalue, long default_RO) ;


void printDateTime();
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];

int PM01Value = 0;        //define PM1.0 value of the air detector module
int PM2_5Value = 0;       //define PM2.5 value of the air detector module
int PM10Value = 0;       //define PM10 value of the air detector module


RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define receiveDatIndex 24

//uint8_t receiveDat[receiveDatIndex]; //receive data from the air detector module
long int sl = 1;

int pin = 35;
//DHT11 dht11(pin);
File myFile;
int sensorValue;
int pin44 = 44;
void setup()
{
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.begin(115200);  // start serial for output
  Serial1.begin(9600);   //set the serial1's Baudrate of the air detector module

  //  Configure LED pin as output
  pinMode(PIN_LED_OUT, OUTPUT);

  // configure input to interrupt
  pinMode(PIN_GATE_IN, INPUT);
  attachInterrupt(IRQ_GATE_IN, soundISR, CHANGE);

  
  Serial.setTimeout(1500);    //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
  pinMode(ss, OUTPUT);
  digitalWrite(ss, HIGH);
  if (!SD.begin(ss))
  {
    Serial.println("initialization failed!");
  }
  else
    Serial.println("Initializtion done");
  delay(150);

  TH02.begin(pin_scl, pin_sda);

  delay(100);


  Serial.println("power on!");
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  Serial.print("Firmware Version = ");
  Serial.println(gas.getVersion());
  pinMode(pin44, OUTPUT);
  
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("Date,  Time, sound, pm1, pm2.5, pm10, No2 , CO , CO2, Temperature, Humidity");

}
void loop()
{//g b g y
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  myFile = SD.open("ENV.CSV", FILE_WRITE);
  if (!myFile) {
    Serial.println("Error in writing!");
  }
  printDateTime();
  //Sound 
  int soundValue = analogRead(PIN_ANALOG_IN);
  Serial.print(soundValue);
  Serial.print(", ");
  myFile.print(soundValue);
  myFile.print(", ");
  if (Serial1.find(0x42)) {  //start to read when detect 0x42
    //Serial.println("Check");
    Serial1.readBytes(buf, LENG);

    if (buf[0] == 0x4d) {
      if (checkValue(buf, LENG)) {
        //Serial.println(buf[0]);
        PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value = transmitPM2_5(buf); //count PM2.5 value of the air detector module
        PM10Value = transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }

  static unsigned long OledTimer = millis();
  if (millis() - OledTimer >= 1000)
  {
    OledTimer = millis();
    
    //Serial.print("PM1.0: ");
    Serial.print(PM01Value);
    myFile.print(PM01Value);
    myFile.print(", ");
    Serial.print(", ");
    //Serial.println("  ug/m3");

    // Serial.print("PM2.5: ");
    Serial.print(PM2_5Value);

    myFile.print(PM2_5Value);
    myFile.print(", ");
    Serial.print(", ");
    //Serial.println("  ug/m3");

    // Serial.print("PM1 0: ");
    Serial.print(PM10Value);
    myFile.print(PM10Value);
    myFile.print(", ");
    Serial.print(", ");

  }
  float c;
#if GAS_EN

  c = gas.measure_NO2();
  //Serial.print("The concentration of NO2 is ");
  if (c >= 0) {
    Serial.print(c,4);
    myFile.print(c,4);
  }
  else Serial.print("invalid");
  Serial.print(", ");
  myFile.print(", ");

  c = gas.measure_CO();
  //Serial.print("The concentration of CO is ");
  if (c >= 0) {
    Serial.print(c, 3);
    myFile.print(c, 3);
  }
  else Serial.print("invalid");
  Serial.print(", ");
  myFile.print(", ");
#endif

  //FOR CO2.........
  sensorValue = analogRead(co2PIN);       // read analog input pin 1
  long resvalue = ((float)22000 * (1023 - sensorValue) / sensorValue);
  Serial.print(mq135_getppm(resvalue, MQ135_RO)); 
  myFile.print(mq135_getppm(resvalue, MQ135_RO));
  long mq135_getro(int resvalue, long ppm);
  Serial.print(", ");
  myFile.print(", ");

#if TH02_EN
  float temper = TH02.ReadTemperature();
  // Serial.println("Temperature: ");
  Serial.print(temper);
  //Serial.println("C\r\n");
  myFile.print(temper);
  Serial.print(", ");
  myFile.print(", ");

  float humidity = TH02.ReadHumidity();
  //Serial.println("Humidity: ");
  Serial.println(humidity);
  myFile.println(humidity);
  //Serial.println("%\r\n\r\n");
#endif


  myFile.close();

  delay(1000);

}

void printDateTime()
{
  DateTime now = rtc.now();
  Serial.print(now.day(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.year(), DEC);
  Serial.print(", ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print(", ");

  myFile.print(now.day(), DEC);
  myFile.print("/");
  myFile.print(now.month(), DEC);
  myFile.print("/");
  myFile.print(now.year(), DEC);
  myFile.print(", ");
  myFile.print(now.hour(), DEC);
  myFile.print(":");
  myFile.print(now.minute(), DEC);
  myFile.print(":");
  myFile.print(now.second(), DEC);
  myFile.print(", ");
}


//Functions
double mq135_getppm(long resvalue, long default_RO)
{
  double ret = 0;
  double validinterval = 0;
  validinterval = resvalue / (double)default_RO;
  //if (validinterval < MQ135_MAXRSRO && validinterval > MQ135_MINRSRO) {
    ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue / default_RO), MQ135_EXPONENT);
  //}
  //ret = ret*1000;
  return (ret);
}
long mq135_getro(long resvalue, long ppm) {
  return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR / ppm) / MQ135_EXPONENT ));
}
char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }
  receiveSum = receiveSum + 0x42;

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1])) //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val = ((thebuf[5] << 8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM2_5Val;
}

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}
