# Goodix GT911 test

Test software for the Goodix GT911 Touch Controller on RadioMaster TX16S

Based on code in EdgeTX https://github.com/EdgeTX/edgetx/blob/2.4/radio/src/targets/horus/tp_gt911.cpp

Hookup between RadioMaster TX16S 6-pin FFC touch controller cable to ST Nucleo-F401RE board:

* Pin 1 - GND - GND
* Pin 2 - I2C SCL - D15, PB8
* Pin 3 - I2C SDA - D14, PB9
* Pin 4 - INT - D12, PA6
* Pin 5 - !RST - D11, PA7
* Pin 6 - VDD - +3.3V
