# HydroSensor

This repository is part of HydroSoil, and contains the latest firmware and some revisions of CAD designs for HydroSensor and HydroSensor Pro. 

HydroSensor's are ultra-low-power devices, running custom embedded firmware designed to communicate with the HydroSoil network.

HydroSensor's are engineered for high-reliability, minimal loss of data, and long battery life. The firmware features:

- Simple cryptographic hashing of REST payloads for API security & replay protection, using built-in RF physical noise registers (`ESP.random()`)
- Tracking of internal battery and soil moisture level, as well as ambient light, temperature and humidity for Pro sensor
- Non-volatile data backups to local disk with tracking of timestamps. During times of unstable network connection, HydroSensor's seemlessly continue readings locally and upload all stored readings when connection is restored, ensuring that virtually no data is lost
- Usage of internal RTC RAM cache to store a running clock in relation to failed network requests, to bypass the lack of a physical RTC, and avoid network/battery overhead of communicating with NTP servers. A CRC32 checksum algorithm is also use d to validate integrity of the RAM on boot cycles
- Dynamic sleep times to intelligently conserve power, calculated based off of environment change delta reading-over-reading, readings compared to normal thresholds, and number of failed requests if server is unreachable
- A "dumb edge, smart cloud" pattern is used in the ecosystem in order to offload processing of data to the HydroSoil cloud, including calculation of dynamic sleep time and conversion of raw sensor readings to usable data.
- RTC RAM cache is used for time data as it is frequently written to in order to preserve silicon wear (limit of about 100,000 writes), and filesystem storage is used for data backup as it is more persistent, and writes are much less frequent - only on network failure

## Tech Stack
The microcontroller the firmware runs on is an Espressif ESP-12F chip, a single-core RISC architecture. The chip is run at 80MHz to preserve battery.
The microcontroller is wired to:
- 1x ADS1115 (a 16-bit ADC over I2C chip, used for readings from the soil moisture sensor)
- 1x SHT-31D (I2C digital temperature and humidity sensor)
- 1x TSL2591 (I2C digital visible & infrared light sensor)

The firmware is written in C++ and utilises PlatformIO core and Espressif 8266 development package. The firmware also uses LittleFS for communication with filesystem storage, WiFiManager for emitting a captive portal when WiFi is not configured, and standard Adafruit libraries for each peripheral used listed above.