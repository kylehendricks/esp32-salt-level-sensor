# esp32-salt-level-sensor
An ESP32 based water softener salt level sensor

Uses the [HC-SR04](http://www.elecfreaks.com/store/download/product/Sensor/HC-SR04/HC-SR04_Ultrasonic_Module_User_Guide.pdf) ultrasonic distance sensor to measure the salt level in the brine tank.

The level is published to an MQTT broker configured in the menuconfig.

## Building

1. Run `make menuconfig` to configure Wifi and MQTT.
2. Run `make flash && make monitor` to flash then tail the ESP32 logs.
