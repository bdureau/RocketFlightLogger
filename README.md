# Rocket Flight logger: AltiMulti, AltiMultiV2, AltiMultiSTM32 or  AltiMultiESP32 (firmware)
This project is part of a complete rocket flight computer. Depending on the board that you have it has 2, 3 or 4 pyro outputs that can be used for the main, the appoge or just simple timers. It has an eeprom that can record up to 25 flights. Those flights can then be read on the PC, Mac or Linux using the Arduino IDE serial port monitor.
I am also working on a Java interface and an Android application that can plot the flight and configure the altimeter.                  

<img src="/pictures/altimulti.JPG" width="49%">  

With a bluetooth module                                                                              
<img src="/pictures/altimulti_bluetooth.JPG" width="49%">       

With a USB cable                                                    
<img src="/pictures/altimulti_usb.JPG" width="49%">      

With a 3DR telemetry module                                             
<img src="/pictures/altimulti_3DRtelemetry.jpg" width="49%">      

Telemetry on your tablet or phone                                              
<img src="/pictures/altimulti_telemetryV2.jpg" width="49%">

# Talking to the altimeter using command line
It is possible to use a terminal to send some sort of AT command to the altimeter. The commands allows you to get the altimeter config, change it or retreive the fligts. Those commands are also used by all the graphical interface.

# Talking to the altimeter using graphical user interfaces
A couple of software can used with the board:
- a Java interface that can be run on any machine that can run Java
- an Android application that can run on any Android device that is running at least Android 4.2. The application is called [BearConsole](https://github.com/bdureau/BearConsole2)

# Communicating with the altimeter board
The AltiMulti board can communicate using the serial connector. On the serial connector you can plug several devices:
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
you will need to go to the config.h file and uncomment only one of the following compilation directive

#define ALTIMULTI // build code for Altimulti ATMega328 board ver1.0

#define ALTIMULTIV2 // build code for Altimulti ATMega328 board ver2.0

#define ALTIMULTISTM32 // build code for Altimulti STM32 board 

#define ALTIMULTIESP32 // build code for Altimulti ESP32 board

#define ALTIMULTIESP32_ACCELERO // build code for Altimulti ESP32 board with an ADXL345 and an ADXL375 accelerometer

#define ALTIMULTIESP32_ACCELERO_375 // build code for Altimulti ESP32 board with an ADXL375 accelerometer

#define ALTIMULTIESP32_ACCELERO_345 // build code for Altimulti ESP32 board with an ADXL345 accelerometer


# Hardware
You can either build the Altimulti board as a kit or use an Arduino Uno/Nano/pro and make a shield. You can also use 3 different type of pressure sensors: BMP085, BMP180 or BMP280. To use the BMP280 sensor, you will need to download the appropriate support library and enable/disable some compilation directives.
The ATMega 328 version has 3 pyro output.
<img src="/pictures/altimultiV2.png" width="49%">

As of version 1.16 you can use an STM32 board which has 4 pyro output and a lot of memory. Plan is to use the additionnal serial ports to have the ability to use GPS modules combined with telemetry.

<img src="/pictures/AltiMultiSTM32.jpg" width="49%">


As of version 1.26 you can use an ESP32 board. Plan is be able to build a more complex altimeter in the futur without the need to have an external bluetooth module

<img src="/pictures/esp32-altimulti.jpg" width="49%">
<img src="/pictures/esp32-altimulti-part2.jpg" width="49%">

As of version 2.0 you can use an ESP32 with accelerometer.
