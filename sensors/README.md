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
* ADC_Sensors : it's easier to read all ADCs at once to avoid open/close conflicts on the bus.
* BME280Sensor
* BME680Sensor
