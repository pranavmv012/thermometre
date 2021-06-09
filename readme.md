In this assignment, you are going to developthe embedded software for implementing the multi-access-point thermometerby using the STM32L432 Nucleo-32 board. For example, 
such a multi-access point thermometercan be used in hospitals to monitor patients’temperatures from different locations.  Your embedded software allows the nurses to monitor the 
patient temperaturesfromthe patient roomby LCD display, in the head-nurse stationthrough Putty Software, and in hallwaysby looking at the LED colorin real time. Your code needs to
Utilize Thermistor for sensing temperature and converting temperature to voltage,
Supply the thermistor with 3.3v,Read thethermistoroutput voltage through PA1 port,
Enable 12-bit ADC on the PA1 portof our Nucleo-32 board,Convert the digitized voltage to temperature,
Display temperature on LCD display and usetheLCEdisplay wiring as the schematic (i.e. PB3 feeding ther/wLCD pin),
Transfer the temperature to the head-nurse station using virtual port communicationvia PA2,
Light upan RGBLED(which is locatedabove the patient room on the hallway) via PA8, PA9, and PA10ports for red, green, and blue colors, respectively
