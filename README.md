# Dehumidifier Sensor
This is a simple arduino project that sends the dehumidifier status to home assistant via MQTT.  It does this by using a photoresistor attached to the LED that flashes when the dehumidifier's bucket is full.

## Dependencies
* [DHT Sensor library](https://github.com/adafruit/DHT-sensor-library)
* ESP8266WiFi - built in
* [PubSubClient](https://github.com/bblanchon/ArduinoJson)
* [ArduinoJson](https://github.com/knolleary/pubsubclient)

## Hardware
* AdaFruit Feather Huzzah ESP8266
* AM2302 sensor (or DHT22)
* AdaFruit ALS-PT19 Analog Light Sensor (or Photoresistor)