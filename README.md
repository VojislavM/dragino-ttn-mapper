# Dragino TTN mapper

This repository contains the code for the [Arduino UNO](https://store.arduino.cc/usa/arduino-uno-rev3) with the [Dragino Lora GPS shield](http://wiki.dragino.com/index.php?title=Lora/GPS_Shield). It is meant as a mobile hardware for [TTN mapper](https://ttnmapper.org/).
There are a few more things to do on this device in order to get the data on [TTN mapper](https://ttnmapper.org/) web site. Plese, folow the instruction on the [TTN mapper](https://ttnmapper.org/faq.php) website.

### Hardware components and connections
Necessary hardware is [Arduino UNO](https://store.arduino.cc/usa/arduino-uno-rev3) and [Dragino Lora GPS shield](http://wiki.dragino.com/index.php?title=Lora/GPS_Shield). You will also need two jumper wires to connect UART to the GPS (see picture below - how to connect the jumper wires).

![image1](https://github.com/VojislavM/draguino-ttn-mapper/blob/master/pics/GPS_Shield_with_Lora_BEE.jpg)

In order to get your data on the [TTN mapper](https://ttnmapper.org/) web site, few steps are required, which are given in the [FAQ](https://ttnmapper.org/faq.php) section of the web site. Just to summarize, as we are going to use node with GPS, our node at least will have to transmit latitude, longitude, altitude and hdop. 
Then we have to send following data to the owner of [TTN mapper](https://ttnmapper.org/) web site:
* Application ID (MQTT username)
* Access Key (MQTT password)
* Device ID (or emphasize if all devices in our application should be listened)
* Region (e.g. EU). Region is the last part of the handler you registered your application to.
* A description of payload format. We can follow the same payload format as the [Sodaq One](https://github.com/SodaqMoja/SodaqOne-UniversalTracker) universal tracker, or use even more compressed format which is used in [this example](https://github.com/jpmeijers/RN2483-Arduino-Library/blob/master/examples/SodaqOne-TTN-Mapper-binary/SodaqOne-TTN-Mapper-binary.ino).
