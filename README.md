# AltiMulti
This project is part of a complete rocket flight computer. The board has 3 pyro output that can be used for the main, the appoge or just simple timers. It has an eeprom that can record up to 25 flights. Those flights can then be read on the PC, Mac or Linux using the Arduino IDE serial port monitor.
I am also working on a Java interface and an Android application that can plot the flight and configure the altimeter. 
<img src="/pictures/altimulti.JPG" width="49%">  

With a bluetooth module                                                                              
<img src="/pictures/altimulti_bluetooth.JPG" width="49%">       

With a USB cable                                                    
<img src="/pictures/altimulti_usb.JPG" width="49%">      

With a 3DR telemetry module                                             
<img src="/pictures/altimulti_3DRtelemetry.jpg" width="49%">      

Telemetry on your tablet or phone                                              
<img src="/pictures/altimulti_telemetry.jpg" width="49%">

# Talking to the altimeter using command line
It is possible to use a terminal to send some sort of AT command to the altimeter. The commands allows you to get the altimeter config, change it or retreive the fligts. Those commands are also used by all the graphical interface.

# Communicating with the altimeter board
The altiMulti board can communicate using the serial connector. On the serial connector you can plug several devices:
- a USB/ttl module
- a bluetooth module
- a 3DR telemetry module
- a GPS module 
etc ....
Any serial module should work. The firmware will have to be changed depending on what you want todo. The standard firmware can work with the first 3 modules.

# Building the code
You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
You have to load the Arduino Uno boot loader to your ATmega328 micro controller. 
Make sure that you download the [support library](https://github.com/bdureau/AltimetersLibs) for the BMP085 sensor and copy them to the Arduino library folder. To compile it you need to choose the Arduino Uno board and the correct USB port.
You will need to use a USB/TTL adapter to connect the altimeter to your computer, refer to the documentation.
