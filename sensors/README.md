# Sensors List

## RPi CPU
These can be included in main() or MasterTask
* Free Memory (Bytes always, to graph easier)
* CPU Temperature (°C)

## ADS1115
Separate sensor tasks:
* 5V Power Monitor (A2)
* 3.3V Power Monitor
* Light Sensor (A3)

## BME280
Include these all as one sensor:
* Temperature (°F)
* Pressure (inHg)
* Rel. Humidity (%)

## BME680
Include these all as one sensor:
* Temperature (°F)
* Pressure (inHg)
* Rel. Humidity (%)
* Gas Resistance (Ohms)
* IAQ Accuracy
* IAQ

---

## Sensor Tasks
* Pwr_5V_Sensor
* Pwr_3p3V_Sensor
* LightSensor
* BME280
* BME680
