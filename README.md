## What can be found here?

These software are for use with the SensEgg-Light sensor ([See what SensEgg is](https://www.arduinoforum.de/arduino-Thread-SensEgg-light-FunkSensor-ATtiny814-nRF24-BME280-NTC)) and a matching receiver. 

The SensEgg-Light sensor is used to record temperature, humidity and air pressure. It features a very low power consumption (quiescent current < 10µA) and can be operated with a 3V CR2032 coin cell for more than one year.

The measurement data is sent to a receiver every minute with an nRF24 transmitter.

The receiver software can be used to process the Sens-Egg data with ioBroker:
[Link to scripts for use with ioBroker](https://github.com/DoImant/iobroker.scripts).

## Pictures

Sensegg Ligt without BME280 and NTC (ATtiny1604 µController).  
![SensEgg Light view 1](https://github.com/DoImant/Stuff/blob/main/SensEgg-Light/se-light.jpg?raw=true)

SensEgglight with NTC and a socket for plugging on a BME280 sensor.   
![SensEgg Light view 2](https://github.com/DoImant/Stuff/blob/main/SensEgg-Light/se-light1.jpg?raw=true)

Quiescent current of a SensEgg-Light sensor
![SensEgg quiescent current](https://github.com/DoImant/Stuff/blob/main/SensEgg-Light/se-quiescent-current.jpg?raw=true)

Arduino Nano Clone as receiver and serial gateway to for example a Raspberry Pi with ioBroker.  
![Receiver (serial gateway)](https://github.com/DoImant/Stuff/blob/main/SensEgg-Light/se-light-receiver.jpg?raw=true)