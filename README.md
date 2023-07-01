# t12-solder-station

![](/document/imgs/1.jpg)
![](/document/imgs/2.jpg)

## Feature
* regulate the temperature on the tip of soldering iron according to the set temperature (200 - 400 degree)
* using a 0.92 inch OLED to display current temperature and set temperature
* auto sleep and wake function according to sensed vibration

## Working principle
1. using PWM wave to control power output to the iron
2. T12 iron tip is made of a type of thermocouple which can be seen as a 8 ohm resistor in series with a temperature-controlled-voltage-source that output voltage in proportion to temperature(in mV), so I use OPA376 opamp to amplify the voltage between two terminals of the iron(the voltage should be measured between the off stage, because when the iron is on, the voltage across it is 24V(so I use a 3.3V zener diode to clamp the voltage feeding into the input port of the opamp in case of destroying it))

## Schematic
![](/document/imgs/sch.png)

## PCB
![](/document/imgs/pcb.jpg)
