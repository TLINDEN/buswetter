# Final Sensor Module

This is the code for the sensore module with all components
included, as small as possible:

```
   text	   data	    bss	    dec	    hex	filename
      0	   6350	      0	   6350	   18ce	build-attinyx41-841/841-tinycore-micronucleus-vcc-sleepmode-bme280-radio-header-test.hex
```

It features:

* Runs most of the time in power save sleep mode
* measures Vcc, temperature, humidity and pressure
* sends the measurements via 433 MHz radio to the ESP32 module


# Calculations
## MAX6301 Watchdog IC

    Vrst = 2.69
    R2   =  940k (470k + 470k)
    R1   = 1220k (1M + 220k)

