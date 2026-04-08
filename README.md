# esphome-abl-emh1
Esphome component for communication with ABL Wallbox eMH1

This is a very basic EspHome component for use with the
ABL eMH1 wallbox.

It reads the current on 3 phases (if supported by the Wallbox) and allows you to set the max current in steps 0f 0,1A.
There is also a switch to enable/disable the charger.
For integration with evcc an evcc-state exists.

Some more output (like the serial number) is available in hidden entities.

The Wallbox is not capable of 1P / 3P switching.

### My hardware
- Wemos D1-Mini, ESP-8266
  ESP GPIO pin 12 is flow-control 
- max485 TTL converter Module, convert serial to RS485
- LM2596 DC / DC Step-Down Converter, converts 12V from connector X10 to 5V
  Regulate output voltage to 5V before connecting to D1-Mini
- Esp Ground connected to the GND pin from connector X10 on the ABL eMH1 circuit board. This helps avoiding noise on the RS485.
- RJ45 plug: pin 1 and 2 connected to SN75176, no other pins connected

### More info, questions
Check out this thread on the Home Assistant community forum:
https://community.home-assistant.io/t/connecting-the-abl-emh1-ev-charger-as-esphome-component

For hardware setup I followed Configuration from DerLev here:
https://community.home-assistant.io/t/connecting-the-abl-emh1-ev-charger-as-esphome-component/705488/29


### Disclaimer
Using this component requires you to connect consumer electronics
inside your ABL eMH1 Wallbox. There's also ~380V in that box, so
please make sure you know what your doing. 

This code was tested by trail and error, it might blow a fuse or two.
Connecting this to your electricity network and your car is your own
responsibility.
