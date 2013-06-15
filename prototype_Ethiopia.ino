#include <JeeLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <GLCD_ST7565.h>
#include <avr/pgmspace.h>

#include "utility/font_clR4x6.h"
#include "utility/font_clR6x6.h"
#include "utility/font_clR9x15.h"
#include "utility/font_helvB18.h"

#define MAX_TEMP 8       // Maximum for acceptable temperatures
#define MIN_TEMP 3       // Minimum for acceptable temperatures
#define MINUTES_OK 3     // Minutes between measurments, when temperatures are within acceptable range
#define MINUTES_DANGER 1 // Minutes between measurments, when temperatures are outside acceptable range
#define AVG_LENGTH 100   // Number of measurements for average computation

#define piezoPin 6
#define alarmPin A2

#define TEMPERATURE_PRECISION 9
#define ONE_WIRE_BUS 5   // Data pin, for sensors
#define TEMP_POWER A1    // Power pin, for sensors
#define MAX_SENSORS 4    // Maximum of sensors which can be connected

// Initialize temperature sensors on onewire.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = {0x28, 0xB8, 0xB7, 0xF1, 0x03, 0x00, 0x00, 0xBC};
DeviceAddress sensor2 = {0x28, 0xD5, 0xF4, 0x0C, 0x04, 0x00, 0x00, 0x04};
DeviceAddress sensor3 = {0x28, 0x73, 0xCA, 0xF1, 0x03, 0x00, 0x00, 0x5A};
DeviceAddress sensor4 = {0x28, 0x27, 0xB2, 0xF1, 0x03, 0x00, 0x00, 0xFB};

DeviceAddress* sensorAddress[4] = {&sensor1, &sensor2, &sensor3, &sensor4};

// Initialize LCD screen;
GLCD_ST7565 glcd;
char buf[10];

ISR(WDT_vect) { // interrupt handler for JeeLabs Sleepy power saving
  Sleepy::watchdogEvent(); 
}

#define myNodeID 24      // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_868MHZ // Frequency of RFM12B module


//########################################################################################################################
//Data Structure to be sent, it is variable in size and we only send 2+n*2 bytes where n is the number of DS18B20 sensors attached
//########################################################################################################################

typedef struct {
  int supplyV;	// Supply voltage
  int temp;	// Temperature reading
  int temp2;	// Temperature 2 reading
  int temp3;	// Temperature 3 reading
  int temp4;	// Temperature 4 reading	
} 
Payload;

Payload temptx;

//########################################################################################################################

int values[4];
int numSensors;
int highTemp, lowTemp; // Highest and lowest temperature.
long accDamage = 0; // Accumulated time at damaging temperatures, in minutes.
int minutes = MINUTES_OK;
long lastReset = -minutes; // Time since last reset, in minutes.

void setup() {
  Serial.begin(9600);
  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  rf12_control(0xC000);			  // Adjust low battery voltage to 2.2V
  rf12_sleep(0);                          // Put the RFM12 to sleep

  PRR = bit(PRTIM1); // only keep timer 0 going
  ADCSRA &= ~ bit(ADEN); // Disable the ADC
  bitSet (PRR, PRADC);   // Power down ADC
  bitClear (ACSR, ACIE); // Disable comparitor interrupts
  bitClear (ACSR, ACD);  // Power down analogue comparitor

  pinMode(TEMP_POWER, OUTPUT);     // set power pin for DS18B20 to output
  pinMode(piezoPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  digitalWrite(TEMP_POWER, HIGH);  // turn sensor power on
  Sleepy::loseSomeTime(50);       // Allow 50ms for the sensor to be ready

  sensors.begin(); 
  sensors.requestTemperatures(); // request temperatures to initialize highest and lowest temperatures
  numSensors=sensors.getDeviceCount();
  Serial.println(numSensors);
  
  highTemp = sensors.getTempCByIndex(0)*100;
  lowTemp = sensors.getTempCByIndex(0)*100;
  
  glcd.begin();
  glcd.backLight(255);
}

void loop() {
  if(numSensors < 4){
    digitalWrite(alarmPin, HIGH);
    playSound(1136, 300);
  } else {
    digitalWrite(alarmPin, LOW);
  }
  
  lastReset += minutes;
  
  pinMode(TEMP_POWER, OUTPUT); // set power pin for DS18B20 to output  
  digitalWrite(TEMP_POWER, HIGH); // turn DS18B20 sensor on
  Sleepy::loseSomeTime(10); // Allow 10ms for the sensor to be ready

  writeTemp(0); 
  writeVcc(); // reads and writes voltage


  digitalWrite(TEMP_POWER, LOW); // turn DS18B20 sensor off
  pinMode(TEMP_POWER, INPUT); // set power pin for DS18B20 to input before sleeping, saves power

  updateHighLow(); // updates highest and lowest values, if needed
  
  rfwrite(); // Send data via RF
  
  if(getHigh() > MAX_TEMP || getLow() < MIN_TEMP) { // Test for hazardous temperature levels
    accDamage += minutes;     // Current timeframe is added to accumulated damage
    minutes = MINUTES_DANGER; // The frequency of measurements is set higher, for precision of the time spend at hazardous levels. 
  } else {
    minutes = MINUTES_OK;
  }
  
  updateLCD();

  for(byte j = 0; j < minutes; j++) {    // Sleeptime in minutes
    Sleepy::loseSomeTime(60000); //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)
  }
}

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
static void rfwrite(){
  rf12_sleep(-1);     //wake up RF module
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &temptx, numSensors*2 + 2); // two bytes for the battery reading, then 2*numSensors for the number of DS18B20s attached to Funky
  rf12_sendWait(2);    //wait for RF to finish sending while in standby mode
  rf12_sleep(0);    //put RF module to sleep
}

//--------------------------------------------------------------------------------------------------
// writes temperatures to payload
//--------------------------------------------------------------------------------------------------
void writeTemp(int testNumber) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  if (!testError() || testNumber == 5) { // assures no errornous measurments are present (error 85)
                                         // maximum of 5 measurments before ignoring error.
    temptx.temp=values[0]=(sensors.getTempC(*sensorAddress[0])*100); // read sensor 1
    if (numSensors>1) temptx.temp2=values[1]=(sensors.getTempC(*sensorAddress[1])*100);
    if (numSensors>2) temptx.temp3=values[2]=(sensors.getTempC(*sensorAddress[2])*100); 
    if (numSensors>3) temptx.temp4=values[3]=(sensors.getTempC(*sensorAddress[3])*100);
  } else {
     writeTemp(testNumber+1); // Recursive calls the function again, as long as 85 error is present or 5 measurments has already been completed.
  }
  Serial.println("ny måling");
  Serial.println(values[0]);
  Serial.println(values[1]);
  Serial.println(values[2]);
  Serial.println(values[3]);
}

boolean testError() { // tests for error 85 readings from each temperature sensor.
  for ( int i = 0; i < numSensors; i++) {
    if (sensors.getTempCByIndex(i) == 85) {
      return true;
    }
  }
  return false;
}

//--------------------------------------------------------------------------------------------------
// Reads and write current voltage
//--------------------------------------------------------------------------------------------------
void writeVcc(){
  bitClear(PRR, PRADC); // power up the ADC
  ADCSRA |= bit(ADEN); // enable the ADC  
  Sleepy::loseSomeTime(10); 
  temptx.supplyV = map(analogRead(6), 0, 1023, 0, 1374);
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
}

//--------------------------------------------------------------------------------------------------
// Updates highest and lowest recorded values
//--------------------------------------------------------------------------------------------------
void updateHighLow(){
  int high = getHigh();
  int low = getLow();
  
  if (high > highTemp) highTemp = high;
  if (low < lowTemp) lowTemp = low;
}

int getHigh() {
  int highest = values[0];
  for(int i = 1; i < numSensors; i++) {
    if(values[i] > highest && values[i] != 85) {
      highest = values[i];
    }
  }
  return highest;
}

int getLow() {
    int lowest = values[0];
  for(int i = 1; i < numSensors; i++) {
    if(values[i] < lowest) {
      lowest = values[i];
    }
  }
  return lowest;
}

int getAverage() {
  int total = values[0];
  for(int i = 1; i < numSensors; i++) {
      total += values[i];
  }
  return total/numSensors;
}

//--------------------------------------------------------------------------------------------------
// Prints interface and values to LCD screen
//--------------------------------------------------------------------------------------------------
void updateLCD() {
  glcd.clear();
  printInterface();
  printValues();
  glcd.refresh();
}

void printInterface() {
  glcd.setFont(font_clR4x6);
  glcd.drawString(0, 0, "CURRENT AVERAGE TEMP");
  glcd.drawString(0, 28, "HIGHEST TEMP");
  glcd.drawString(65, 28, "LOWEST TEMP");
  glcd.drawString(0, 47, "TIME SINCE LAST RESET");
  
  glcd.drawLine(0, 26, 128, 26, WHITE);
  glcd.drawLine(0, 45, 128, 45, WHITE);
  glcd.drawLine(63, 26, 63, 45, WHITE);
  glcd.drawLine(82, 0, 82, 26, WHITE);
  
  if(numSensors < 4){
    for(int i=0; i < 3; i++){
      glcd.drawLine(120-i, 3, 90-i, 24, WHITE);
      glcd.drawLine(90-i, 3, 120-i, 24, WHITE);
    }
  } else {// OK
    for(int i=0; i < 3; i++){
      glcd.drawLine(120-i, 3, 100-i, 24, WHITE);
      glcd.drawLine(97-i, 14, 100-i, 24, WHITE);
    }
  }
  
  // UNICEF DTU
  glcd.drawString(103, 50, "UNICEF");
  glcd.drawString(108, 56, "DTU");
  glcd.drawLine(99, 45, 99, 68, WHITE);
  
}

void printValues() {
  glcd.setFont(font_helvB18);
  toString(getAverage());
  glcd.drawString(0, 6, buf);
  glcd.setFont(font_clR9x15);
  toString(highTemp);
  glcd.drawString(0, 33, buf);
  toString(lowTemp);
  glcd.drawString(64, 33, buf);
  
  int d = lastReset/60/24;
  int h = (lastReset/60)%24;
  int m = lastReset%60;
  
  sprintf(buf, "%dd %dh %dm", d, h, m);
  glcd.drawString(0,52, buf);
}

void toString(int val){
  sprintf(buf, "%4d.%d", val/100, abs(val-(val/100*100)));
}

//--------------------------------------------------------------------------------------------------
// Controls for piezo element
//--------------------------------------------------------------------------------------------------
void playSound(int tone, int duration) {
    for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(piezoPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(piezoPin, LOW);
    delayMicroseconds(tone);
  }
}



