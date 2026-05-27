/**
 * sensor.hpp
 * Low-power embedded firmware for HydroSensors
 */

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h> // For OTA updates from server
#include <Adafruit_ADS1X15.h> // Digital I2C ADC driver
#include <ArduinoJson.h>
#include <LittleFS.h>

#ifdef HARDWARE_PRO
#include <Adafruit_SHT31.h> // For temp/humidity
#include <Adafruit_TSL2591.h> // For ambient light sensor
#endif // HARDWARE_PRO

class Sensor {
public:
    Sensor();
    ~Sensor() = default;

    // Initialises hardware, peripherals, and network
    void init();
    
    // Periodically ingests hardware telemetry, creates POST payload and dispatches to server
    void dispatchData();

private:
    // Cache for storing thresholds and last readings
    struct RtcCache {
        uint32_t checksum; // Checksum to verify no corrupted data from the RAM
        uint32_t clock; // Accumulated time to calculate difference (only used for backlog)

        // Last sensor readings for calculating delta
        bool lastReadingsPresent;
        float lastMoisture;
#ifdef HARDWARE_PRO
        float lastTemp;
        float lastLux;
#endif // HARDWARE_PRO
        
        // Threshold targets recieved from server
        bool thresholdsPresent;
        float moistureLow;
        float moistureHigh;
#ifdef HARDWARE_PRO
        float tempLow;
        float tempHigh;
        float luxLow;
        float luxHigh;
#endif // HARDWARE_PRO
    };

    // Network configuration
    static constexpr const char* AP_NAME = "HydroSensor-CONNECT"; // Name of the setup access point (fixed so iOS app can automatically connect to it)
#ifdef HARDWARE_PRO
    static constexpr const char* FIRMWARE_VERSION = "P-2.0.0"; // Firmware version
    static constexpr const char* HTTP_USER_AGENT = "HydroSensorPro/2.0"; // User agent for server endpoints
#else
    static constexpr const char* FIRMWARE_VERSION = "R-2.0.0"; // Firmware version
    static constexpr const char* HTTP_USER_AGENT = "HydroSensor/2.0"; // User agent for server endpoints
#endif // HARDWARE_PRO
    static constexpr const char* DATA_HOST = "https://api.hydrosoil.tk/sensor/telemetry"; // Server post data endpoint
    static constexpr const char* UPDATE_HOST = "https://api.hydrosoil.tk/sensor/update"; // Server OTA update endpoint
    static constexpr const char* BUFFER_FILE = "/buffer.txt"; // LittleFS file path of the buffer file, in case network fails

    // Timer interval & cache config
    static constexpr uint32_t BASE_INTERVAL = 5 * 60 * 1000000; // (us) Stanadrd time between dispatching telemetry (5 minutes)
    uint32_t m_sleepInterval = BASE_INTERVAL; // (us) Current calculated sleep interval based on data
    static constexpr uint32_t CACHE_THRESHOLD = 5 * 60; // (s) Max time that RTC Cache is valid (30 minutes)

    // Hardware pins
    static constexpr uint8_t LED_GPIO = 14; // Pin number of LED status indicator
    static constexpr uint8_t MOISTURE_ADC_CHANNEL = 0; // Pin number of soil moisture sensor

    // Hardware periphs
    Adafruit_ADS1115 m_ads; // ADC Instance
#ifdef HARDWARE_PRO
    Adafruit_SHT31 m_sht31; // Temp/humidity sensor instance
    Adafruit_TSL2591 m_tsl2591; // Light sensor instance
#endif // HARDWARE_PRO

    // Hardware flags
    bool m_adsOnline = false;
#ifdef HARDWARE_PRO
    bool m_shtOnline = false;
    bool m_tslOnline = false;
#endif

    // Network clients
    WiFiClientSecure m_secureClient; // HTTPS WiFi client
    WiFiClient m_updateClient; // HTTP WiFi client
    RtcCache m_cache;
    
    // Main functions
    bool checkUpdates(); // Checks and installs update if needed
    void enterDeepSleep(); // Instantiates chip deep sleep

    // Auth functions
    String generateRequestId() const; // Generate random request ID
    String generateApiKey(const String& requestId) const; // Generate API key based off of request ID

    // RTC cache functions
    uint32_t calculateChecksum(const uint8_t* data, size_t length) const; // Calculate CRC32 checksum
    void readCache(); // Read data from RTC memory
    void writeCache(); // Write last readings and thresholds to RTC memory
    bool parseNewThresholds(Stream& jsonStream); // Parse response body of data POST request to write new thresholds to cache
    
    // Offline buffer functions
    void appendToBuffer(const String& dataLine); // Appends sensor data to buffer (when network call fails)
    bool dispatchBuffer(HTTPClient& http); // Sends all backed up data to server and clears it

    // Interrupt
    static void IRAM_ATTR onTimerISR();
    static void configModeCallback(WiFiManager* wifiManager);
};

#endif // SENSOR_HPP
