# STM32 OPEN VENTILATOR

An open-source emergency ventilator design around the STM32F407 microcontroller. 

### Prerequisites

STM32Cube_FW_F4_V1.21.0 and System Workbench for STM32. The software was built on windows.

## Getting Started

Download all the files. Adjust to you ambu's dimensions and 3D print the ones in the STM32OV folder. Get your hands on 3+ servos with >= 20kg*cm torque that support an update at 200Hz. Get your hand on a shield or build it according to the schematics.

For the software, it runs on en.stm32cubef4\STM32Cube_FW_F4_V1.21.0. Set up your working directory in \Projects\STM32F4-Discovery\Examples\UART\UART_Ventilator. This will allow the compiler to link and compile external driver dependencies. Use System Workbench for STM32 as working environement.

## Hardware setup

![GitHub Logo](/images/ventilator_setup.png)

The plot on the upper right side can be obtained by using arduino's serial plotter on a computer connected on the serial port.

## Current limitations

* TODO Shield PCB
* TODO air mixing chamber to be added and O2 sensors to measure and control input oxygen concentration.
* TODO air warmer and humidifier.
* TODO a proper back-pressure PEEP valve controllable by other servos on the output.
* TODO air filter the output

## Software development

* TODO plot flow
* TODO finalize PRVC mode
* TODO detect servo failures and create a certain recognizable alarm sound. Has to adapt strength to compensate
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
