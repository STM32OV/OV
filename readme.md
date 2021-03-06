# STM32 OPEN VENTILATOR

An open-source emergency ventilator design around the STM32F407 microcontroller. 

### Prerequisites

STM32Cube_FW_F4_V1.21.0 and System Workbench for STM32. The software was built on windows.

## Advantages

Using the STM32F407 micrcontroller has some advantages over many others:

* 16 bit timers with more than 8x PWM outputs. This allows us to control servos without sacrificing precion while usin 200Hz pwm.
* Supports analog (12 bit ADC) sensors, i2c and spi.
* 186 Mhz Arm core, which is enough for most demanding algorithms as well as an embedded display on the controller.
* Available on a pcb with crystal clock and in high quantities.
* Using an serial connection to a laptop and a remote OpenGL display appliation allows us to separate the realtime and critical portion of the software from the display.

Additionally, pressure and current consumption of the servos is monitored, which allows us to detect defects early and ring alarms.

## Getting Started

Download all the files. Adjust to you ambu's dimensions and 3D print the ones in the STM32OV folder. Get your hands on the part list. Follow the assembly tutorial.

For the software, it runs on en.stm32cubef4\STM32Cube_FW_F4_V1.21.0. Set up your working directory in \Projects\STM32F4-Discovery\Examples\UART\UART_Ventilator. This will allow the compiler to link and compile external driver dependencies. Use System Workbench for STM32 as working environement.

## Parts
 
* Pressure sensor: MPXV5004DP https://www.digikey.ca/products/en/sensors-transducers/pressure-sensors-transducers/512?k=MPXV5004DP
* Servos: Any brushless servo with at least 25kg*cm torque and 200Hz update rate. 
* STM32F407 Disco board: https://www.digikey.ca/product-detail/en/stmicroelectronics/STM32F407G-DISC1/497-16287-ND/5824404
* Adjustable PEEP valve: Something like this https://www.ebay.ca/itm/Ambu-PEEP-Ventil-Valve-10-fur-Beatmungsbeutel-Beatmung-Reanimation/293532262144?hash=item4457e2df00:g:DFMAAOSw6IdcQypx
* Electronic humidifer and warmer: https://www.medicalexpo.com/prod/wilamed/product-70540-428587.html
* Oxygen sensor: https://www.sgxsensortech.com/content/uploads/2014/07/A1A-EC410-EC410-Datasheet-V4.pdf
* STM32F407 shield: get in touch with us we'll make it available.

## Hardware setup 

![GitHub Logo](/images/ventilator_setup.png)

The plot on the upper right side can be obtained by using arduino's serial plotter on a computer connected on the serial port.
Note: the pressure sensors have 2 ports. Use the upper one, as it's specified in the datasheet that it's the one that is protected from ingress or water damage. Connect the 3 active pins to a servo connector using the pinout Sgnl, 5V, Gnd.

## Hardware development

* TODO Shield PCB
* TODO air mixing chamber to be added and O2 sensors to measure and control input oxygen concentration.
* TODO air warmer and humidifier.
* TODO a proper back-pressure PEEP valve controllable by other servos on the output.
* TODO air filter on the output
* TODO battery backup

## Software development

* TODO plot flow
* TODO finalize PRVC mode
* TODO detect servo failures and create a certain alarm sound. Adapt strength to compensate
* TODO display measured values on the upper side of the LCD display
* TODO create an OpenGL display on the computer that can display parameters and plot graphs at high resolutions

## Online Resources
* https://www.github.com/stm32openv
* https://www.twich.tv/stm32openv
* https://www.youtube.com/watch?v=p47dtsTrSBI
* Support us: bc1qvfh9pt93a0lfqc73k87yw9q587tgxdwmyswrv8

## Contributing

We are looking for contributors that are in the field and need to use this product right now.

## Authors

STM32 Open Ventilator - Initial work

## License

This project is licensed under the MIT License - see the [license.md](license.md) file for details

## Acknowledgments

* Special thanks to ST7565 LCD library! Copyright (C) 2010 Limor Fried, Adafruit Industries. The library was modified to provide 1 grayscale level.
* Special thanks to ST Microelectronics for the SDK and all the example code.

