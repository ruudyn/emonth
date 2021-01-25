/*
EmonTH Low Power DHT22 Humidity & Temperature & Pulse counting Node Example.

Checkes at startup for presence of a DHT22 (temp + humidity) sensor.
If it finds only a DHT22 then both temperature and humidity values will be
obtained from this sesor.

Technical hardware documentation wiki:
http://wiki.openenergymonitor.org/index.php?title=EmonTH

Part of the openenergymonitor.org project
Licence: GNU GPL V3

Forked by: ruudyn
Original author: Glyn Hudson
Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work

THIS SKETCH REQUIRES:

Libraries in the standard arduino libraries folder:
  - JeeLib		    https://github.com/jcw/jeelib
  - DHT22 Sensor Library    https://github.com/adafruit/DHT-sensor-library
      Be sure to rename the sketch folder to remove the '-'
  - OneWire library	    http://www.pjrc.com/teensy/td_libs_OneWire.html
  - DallasTemperature	    http://download.milesburton.com/Arduino/
    MaximTemperature/DallasTemperature_LATEST.zip

Recommended node ID allocation:
----------------------------------------------------------------------
  -ID-	-Node Type-
  0	- Special allocation in JeeLib RFM12 driver - Reserved for OOK use
  1-4   - Control nodes
  5-10	- Energy monitoring nodes
  11-14	- Un-assigned
  15-16	- Base Station & logging nodes
  17-30	- Environmental sensing nodes (temperature humidity etc.)
  31	- Special allocation in JeeLib RFM12 driver - Node31 can
          communicate with nodes on any network group
----------------------------------------------------------------------
 
Change log:
----------------------------------------------------------------------
v2.1 - Branched from emonTH_DHT22_DS18B20 example, first version of pulse
       counting version
v2.2 - 60s RF transmit period now uses timer1, pulse events are decoupled
       from RF transmit
v2.3 - Rebuilt based on low power pulse counting code by Eric Amann:
       http://openenergymonitor.org/emon/node/10834
v2.4 - 5 min default transmisson time = 300 ms
v2.3 - (12/10/14) Don't flash LED on RF transmission to save power
V2.4 - (15/10/15) Activate pulse count pin input pullup to stop spurious
       pulses when no sensor connected
V2.5 - (23/10/15) Default nodeID 23 to enable new emonHub.conf decoder for
       pulseCount packet structure
V2.6 - (24/10/15) Tweak RF transmission timing to help reduce RF packet loss
V2.7 - (04/01/17) Reduce runtime checks and code size of non-debug build
V2.8 - (22/01/21) Add real time power counting from pulses and remove external
       temperature sensor
----------------------------------------------------------------------

emonhub.conf node decoder:
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/
configuration.md

[[23]]
  nodename = emonTH_5
  firmware = V2.x_emonTH_DHT22_DS18B20_RFM69CW_Pulse
  hardware = emonTH_(Node_ID_Switch_DIP1:OFF_DIP2:OFF)
  [[[rx]]]
     names = temperature, external temperature, humidity, battery,
             pulseCount
     datacodes = h,h,h,h,L
     scales = 0.1,0.1,0.1,0.1,1
     units = C,C,%,V,p
*/

// Firmware version multiplied by 10 e,g 16 = V1.6
const byte version = 28;
// These variables control the transmit timing of the emonTH, value in
// milliseconds
const unsigned long WDT_PERIOD = 80;
// Data sent after WDT_MAX_NUMBER periods of WDT_PERIOD ms without pulses.
// This needs to be about 5 seconds less than the record interval in emoncms.
// 320 * 80 = 25 600 ms = 25.6 seconds.
// 690 * 80 = 55 200 ms = 55.2 seconds.
const unsigned long WDT_MAX_NUMBER = 320;

const  unsigned long PULSE_MAX_DURATION = 50;


// Set to 1 if using RFM69CW or 0 is using RFM12B
#define RF69_COMPAT 1
// https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14
#include <JeeLib.h>

// Set to 1 to few debug serial output, turning debug off increases
// battery life
const boolean debug=0;

// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ.
// You should use the one matching the module you have.
#define RF_freq RF12_433MHZ
// EmonTH temperature RFM12B node ID - should be unique on network
int nodeID = 23;
// EmonTH RFM12B wireless network group - needs to be same as emonBase
// and emonGLCD.
const int networkGroup = 210;

// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

// Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be
// put into sleep mode inbetween readings to reduce power consumption
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// Hardwired emonTH pin allocations
const byte DHT22_PWR=      6;
const byte LED=            9;
const byte BATT_ADC=       1;
const byte DIP_switch1=    7;
const byte DIP_switch2=    8;
// INT 1 / Dig 3 Screw Terminal Block Number 4 on emonTH V1.5
// Change to INT0 DIG2 on emonTH V1.4
const byte pulse_countINT= 1;
// INT 1 / Dig 3 Screw Terminal Block Number 4 on emonTH V1.5
// Change to INT0 DIG2 on emonTH V1.4
const byte pulse_count_pin=3;
#define ONE_WIRE_BUS       19
#define DHTPIN             18

// EmonTH DHT22 data pin

// Humidity code adapted from ladyada' example
// Uncomment whatever type you're using!
// DHT 11
// #define DHTTYPE DHT11
// DHT 22 (AM2302)
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
// Create flag variable to store presence of DHT22
boolean DHT22_status;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// RFM12B RF payload datastructure:
typedef struct {
  int temp;
  int humidity;
  int battery;
  unsigned long pulsecount;
  unsigned int sinceupdate;
  unsigned int power;
} Payload;
Payload emonth;

int numSensors;
// Addresses of sensors, MAX 4!
// 8 bytes per address
byte allAddress [4][8];

volatile unsigned long pulseCount;
unsigned long WDT_number;
boolean p;

// Time in milliseconds at last and current rf send.
unsigned long timelast = 0;
unsigned long timenow = 0;
unsigned long timedelta;

//####################################################################
void setup() {
//####################################################################

  // Status LED on
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);

  // Read dip switch positions - low when switched on (default off - pulled
  // up high).
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  boolean DIP1 = digitalRead(DIP_switch1);
  boolean DIP2 = digitalRead(DIP_switch2);

  if ((DIP1 == HIGH) && (DIP2 == HIGH)) nodeID=nodeID;
  if ((DIP1 == LOW) && (DIP2 == HIGH)) nodeID=nodeID+1;
  if ((DIP1 == HIGH) && (DIP2 == LOW)) nodeID=nodeID+2;
  if ((DIP1 == LOW) && (DIP2 == LOW)) nodeID=nodeID+3;

  // Initialize RFM12B
  rf12_initialize(nodeID, RF_freq, networkGroup);

  rf12_sleep(RF12_SLEEP);
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.print(DIP1); Serial.println(DIP2);
    Serial.println("OpenEnergyMonitor.org");
    Serial.print("emonTH - Firmware V"); Serial.println(version*0.1);
    #if (RF69_COMPAT)
      Serial.println("RFM69CW Init> ");
    #else
      Serial.println("RFM12B Init> ");
    #endif
    Serial.print("Node: ");
    Serial.print(nodeID);
    Serial.print(" Freq: ");
    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz");
    Serial.print(" Network: ");
    Serial.println(networkGroup);
    delay(100);
  }

  pinMode(DHT22_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);
  digitalWrite(DHT22_PWR,LOW);
  pinMode(pulse_count_pin, INPUT_PULLUP);

  //##################################################################
  // Power Save  - turn off what we don't need
  // http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //##################################################################
  // Disable Analog comparator
  ACSR |= (1 << ACD);
  // Disable serial UART
  if (debug==0) power_usart0_disable();
  // Disable the Two Wire Interface module
  power_twi_disable();
  // Don't disable, necessary for the DS18B20 library
  // power_timer0_disable();
  power_timer1_disable();
  power_spi_disable();

  //##################################################################
  // Test for presence of DHT22
  //##################################################################
  // Power up
  digitalWrite(DHT22_PWR,HIGH);
  // Wait 2 seconds for DH22 to warm up
  dodelay(2000);
  dht.begin();
  // Read Humidity
  float h = dht.readHumidity();
  // Read Temperature
  float t = dht.readTemperature();
  // Power down
  digitalWrite(DHT22_PWR,LOW);

  // Check if returns are valid,
  // if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h))
  {
    Sleepy::loseSomeTime(1500);
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h))
    {
      if (debug==1) Serial.println("No DHT22");
      DHT22_status=0;
    }
  }
  else
  {
    DHT22_status=1;
    if (debug==1) Serial.println("Detected DHT22");
  }

  //###############################################################

  digitalWrite(LED,LOW);

  emonth.pulsecount = 0;
  pulseCount = 0;
  WDT_number=720;
  p = 0;

  timelast = millis();
  attachInterrupt(pulse_countINT, onPulse, RISING);
}

// End of setup
//####################################################################


//####################################################################
void loop()
//####################################################################
{

  if (p) {
    Sleepy::loseSomeTime(PULSE_MAX_DURATION);
    p = 0;
  }

  if (Sleepy::loseSomeTime(WDT_PERIOD)==1) {
    WDT_number++;
  }

  if (WDT_number>=WDT_MAX_NUMBER)
  {
    cli();
    emonth.pulsecount += (unsigned long) pulseCount;
    
    timenow = millis();
    // By using unsigned long we avoid millis() overflow after approx. 50 days.
    timedelta = timenow - timelast;
    emonth.sinceupdate = (unsigned int) timedelta;

    // Calculate power in watts:
    emonth.power = (unsigned int) ((pulseCount * 3600000) / timedelta);

    // Reset values
    timelast = timenow;
    pulseCount = 0;
    sei();

    if (DHT22_status==1)
    {
      // Send the command to get temperatures
      digitalWrite(DHT22_PWR,HIGH);
      // Sleep for 1.5 - 2 seconds to allow sensor to warm up
      dodelay(2000);
      // Note: Sensor readings may also be up to 2 seconds 'old' (its a very
      // slow sensor).
      emonth.humidity = ((dht.readHumidity())*10);

      float temp=(dht.readTemperature());
      if ((temp<85.0) && (temp>-40.0)) emonth.temp = (temp*10);

      digitalWrite(DHT22_PWR,LOW);
    }

    // Read battery voltage, convert ADC to volts x1000
    emonth.battery=int(analogRead(BATT_ADC)*3.222);

    if (debug==1)
    {
      if (DHT22_status)
      {
        Serial.print("DHT22 Temperature: ");
        Serial.print(emonth.temp/10.0);
        Serial.print("C, DHT22 Humidity: ");
        Serial.print(emonth.humidity/10.0);
        Serial.print("%, ");
      }

      Serial.print("Battery voltage: ");
      Serial.print(emonth.battery/1000.0);
      Serial.print("V, Pulse count: ");
      Serial.print(emonth.pulsecount);
      Serial.println("n");

      delay(100);
    }

    power_spi_enable();
    rf12_sleep(RF12_WAKEUP);
    dodelay(100);
    rf12_sendNow(0, &emonth, sizeof emonth);
    // Set the sync mode to 2 if the fuses are still the Arduino default.
    // Mode 3 (full powerdown) can only be used with 258 CK startup fuses.
    rf12_sendWait(2);
    rf12_sleep(RF12_SLEEP);
    dodelay(100);
    power_spi_disable();

    WDT_number=0;
  }

}
// End loop
//####################################################################


void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;

  // JeeLabs power save function: Enter low power mode for x seconds (valid
  // range 16-65000 ms).
  Sleepy::loseSomeTime(ms);

  // Restore ADC state
  ADCSRA=oldADCSRA;
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

// The interrupt routine. Runs each time a rising edge of a pulse is
// detected.
void onPulse()
{
  // Flag for new pulse set to true
  p = 1;
  // Number of pulses since the last RF sent
  pulseCount++;
}
