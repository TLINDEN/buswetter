# Buswetter
A tiny weather station for my camper van

# Main Unit

The main  unit consists  of an  Lolin Wemos D323  board with  an ESP32
MCU. It will be powered by an  3.7V LiPo akku. The Wemos board already
contains the recharging  and protection circuitry for  such akkus. The
Unit will also be  connected to the board 12V net via  an LDO and some
protective components.  However,  the board 12V will  not be available
all the  time, since  we turn it  only on if  I need  it or if  we are
connected to land mains.

The unit will also contain a simple RF 433Mhz receiver and a real time
clock module. The display will be an ePaper display.

The primary purpose will be to display the time and an icon indicating
a weather forecast for the next few hours.

# Sensor Unit

The sensor  unit will  be a  1xAA battery powered  Attiny841 with  an Bosch
BME280 sensor module and a RF 433 Mhz sender module. It will be placed
outside the  bus, somewhere  below, e.g. inside  the reserve  wheel or
inside the step below the door.

It  will stay  in power  save sleep  mode most  of the  time, wake  up
collect sensor data, battery voltage, and send it to the main unit.

# Build Environments

I do  not use  any graphical  development environments.  Everything is
being built using Makefiles.

For the ESP32 I use:

* [Espressiv SDK](https://readthedocs.com/projects/espressif-esp-idf/)
* [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32/)
* [esp-idf-template](https://github.com/espressif/esp-idf-template)
    
For the Attiny841 I use:

* [Arduino Makefile](https://github.com/sudar/Arduino-Makefile)
* [ATTiny Core](https://github.com/SpenceKonde/ATTinyCore)

# Libraries

I use the following libraries:

* [RF Transmitter](https://github.com/zeitgeist87/RFTransmitter)
* [Tiny BME280 Library](https://github.com/fabyte/Tiny_BME280_Arduino_Library)
* [GxEPD ePaper library](https://github.com/ZinggJM/GxEPD)
* original Arduino libraries: SPE, GFX, Wire
    
I wrote the following libraries myself:

* [ESP Pin Change Interrupt Handler](mainuinit/ESPPinChangeInterruptHandler/) [ported to ESP32](https://github.com/zeitgeist87/RFReceiver)
* [Attiny841 Sleep Mode](sensor/841sleep.h)
* [Attiny841 read VCC](sensor/841vcc.h)
* [Wire transfer data conversion](sensor/data2wire.h)
    
