/* -*-c++-*-

   Attiny841 based sensor module,  measures Vcc, Temperature, Pressure
   and  Humidity, sends  it to  the weather  display station,  goes to
   sleep, repeat.

   We're using a Bosch BME280 sensor module and a MAX6301 monitoring IC.

  Pins:
  ./hardware/ATTinyCore/avr/cores/tinymodern/core_pins.h
  Caution: reverse Pinout: PA3 => Pin 3

  Nanite841 Pinout from above:

              button
SDO/MISO PA5          PA6 SDI/MOSI
     SCK PA4          PA7
     CS  PA3          PB2
     RX0 PA2          PB2
     TX0 PA1          PB1
         PA0          PB0
         GND          VCC
               usb

  Bare Attiny841 Pinout, pin1 upper left corner, Arduino PIN numbers outside

            VCC 1 o      14 GND
10          PB0 2        13 PA0              0
 9          PB1 3        12 PA1 TX0          1
11      RST PB3 4        11 PA2 RX0          2
 8          PB2 5        10 PA3              3
 7          PA7 6         9 PA4 SCK/SCL/RX1  4
 6 SDA/MOSI PA6 7         8 PA5 MISO/TX1    5

  Bosch BME280 Breakout from above pin header left:

    VCC
    GND
    SCL  => SCK
    SDI  => MOSI
    CSB  => PA3/3
    SDO  => MISO
  
 */

#include <avr/io.h>
#include <util/delay.h>
#include "841vcc.h"
#include "841sleep.h"
#include "data2wire.h"

#define TINY_BME280_SPI
#include <TinyBME280.h>

// https://andreasrohner.at/posts/Electronics/New-Arduino-library-for-433-Mhz-AM-Radio-Modules/
#include <RFTransmitter.h>

#define NODE_ID       1
#define RF_OUTPUT     7 // RF Out
#define CS            3
#define DELAY         5000

// internal sleep repeats
#define WDTREPEATS    3

// external watchdog
#define WDI           0  // watchdog input pin, send pulse
#define WDS           1  // watchdog mode pin, set high during sleep

#ifdef DEV
  #define LED           2 // LED_BUILTIN  // PORTB2
#endif

static tiny::BME280 sensor;
RFTransmitter transmitter(RF_OUTPUT, NODE_ID);
const long InternalReferenceVoltage = 1083L;

#define SET_OUTPUT(pin) DDRB  |=  (1 << pin)
#define SET_HIGH(pin)   PORTB |=  (1 << pin)
#define SET_LOW(pin)    PORTB &= ~(1 << pin)

typedef struct _measurements_t {
  uint32_t pres;
  uint32_t humidity;
  int32_t  temp;
  uint16_t vcc;
} measurements_t;


void wdi_pulse() {
  // send a short pulse to max6301 WDI pin to signal we're alive
  SET_HIGH(WDI);
  delay_ns(30);
  SET_LOW(WDI);
}

#ifdef DEV
void print_asufloat(uint32_t val, uint16_t factor) {
  Serial.print(val / factor);
  Serial.print(".");
  Serial.print(val % factor);
}

void print_asifloat(int32_t val, uint16_t factor) {
  Serial.print(val / factor);
  Serial.print(".");
  Serial.print(val % factor);
}
#endif

void measure() {
  measurements_t ms;
  byte sendms[sizeof(measurements_t)];

#ifdef DEV  
  SET_LOW(LED);
#ENDIF

  ms.temp     = sensor.readFixedTempC();
  ms.humidity = sensor.readFixedHumidity();
  ms.pres     = sensor.readFixedPressure();

  adc_enable();
  adc_start();
  ms.vcc = adc_get_adcw();
  adc_disable();

#ifdef DEV
  Serial.print("Voltage: ");
  Serial.println(ms.vcc);

  Serial.print("     Temperature: ");
  print_asifloat(ms.temp, 100);
  Serial.println(" Grad C");

  Serial.print("        Pressure: ");
  print_asufloat(ms.pres, 100);
  Serial.println(" hPa");

  Serial.print("        Humidity: ");
  print_asufloat(ms.humidity, 1000);
  Serial.println(" %");
#endif

  data32_to_wire(ms.pres,     &sendms[0]);
  data32_to_wire(ms.humidity, &sendms[sizeof(ms.humidity) + 1]);
  data32_to_wire(ms.temp,     &sendms[sizeof(ms.temp)     + 1]);
  data16_to_wire(ms.vcc,      &sendms[sizeof(ms.vcc)      + 1]);
  
  transmitter.send((byte *)sendms, sizeof(sendms));

#ifdef DEV
  SET_HIGH(LED);
#endif
}

void halt() {
  while(1);
}


void setup() {
  // initialize max6301
  SET_OUTPUT(WDI);
  SET_OUTPUT(WDS);
  SET_LOW(WDI);
  SET_LOW(WDS);

#ifdef DEV
  SET_OUTPUT(LED);
  Serial.begin(115200);
  Serial.println("init");
#endif

  delay(4000);

  adc_setup_vcc_measurement();
  wdi_pulse();

  if(sensor.beginSPI(CS) == false) {
#ifdef DEV
    Serial.println("Sensor BME280 connect failed, check wiring!");
#endif
    halt();
  }

  sleep_setup();
}

void loop() {
  uint8_t i;

  // signal alive
  wdi_pulse();

  // do the work
  measure();

  // go to sleep
  for(i=0; i<WDTREPEATS; i++) {
    delay(DELAY);
#ifdef DEV
    Serial.print("----- ENTER SLEEP: ");
    Serial.println(i);
#endif

    // set WDS high which multiplies WDTIMEOUT by 500
    SET_HIGH(WDS);

    // actual sleep
    sleep_enter();

    // awake, set WDS low for configured WDTIMEOUT
    SET_LOW(WDS);

#ifdef DEV
    Serial.println("----- LEAVE SLEEP");
#endif
  }

#ifdef DEV
  Serial.println("-- DONE");
#endif
}

sleep_vect();

