# EnvironSensor
A simple environmental sensor and MQTT logger.

This is a very basic project to read and log environmental data
to a MQTT broker. At present two architectures are supported:
ESP8266 and ESP32. Three different sensors are also supported:

1. BME280
2. DHT-22
3. CCS811

These sensors record and report the following values:

1. Temperature.
2. Relative humidity.
3. Air pressure
4. eCO2
5. TVOC

There are two versions of this program:

1. Arduino
2. Mongoose OS
