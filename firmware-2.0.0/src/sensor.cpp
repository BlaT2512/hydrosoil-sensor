/**
 * sensor.cpp
 * Manages update checking, collection of sensor data, and communication to server
 */

#include "sensor.hpp"

#define DEBUG_MODE false // Outputs logs to serial if true
#define LOG_SERIAL if (DEBUG_MODE) Serial

ADC_MODE(ADC_VCC);

#ifdef HARDWARE_PRO
Sensor::Sensor() : m_ads(), m_sht31(), m_tsl2591() {}
#else
Sensor::Sensor() : m_ads() {}
#endif // HARDWARE_PRO

// MARK: RTC Memory
uint32_t Sensor::calculateChecksum(const uint8_t* data, size_t length) const {
    // Standard calculation of CRC32
    // Inspired by https://stackoverflow.com/questions/31585116/crc32-calculation-with-crc-hash-at-the-beginning-of-the-message-in-c
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

void Sensor::readCache() {
    if (ESP.rtcUserMemoryRead(64, (uint32_t*)&m_cache, sizeof(m_cache))) {
        uint32_t checksum = calculateChecksum(((uint8_t*)&m_cache) + 4, sizeof(m_cache) - 4);
        if (checksum != m_cache.checksum) {
            m_cache.lastReadingsPresent = false;
            m_cache.thresholdsPresent = false;
            m_cache.clock = 0;
        } else {
            m_cache.clock += (m_sleepInterval / 1000000); // Accumulate time just slept
            LOG_SERIAL.println("Using RTC cache");
        }
    }
}

void Sensor::writeCache() {
    m_cache.checksum = calculateChecksum(((uint8_t*)&m_cache) + 4, sizeof(m_cache) - 4);
    ESP.rtcUserMemoryWrite(64, (uint32_t*)&m_cache, sizeof(m_cache));
}

bool Sensor::parseNewThresholds(Stream& jsonStream) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStream);

    if (error) {
        LOG_SERIAL.printf("Failed to parse JSON response: %s\n", error.c_str());
        return false;
    }

    if (doc["thresholdsPresent"]) {
        m_cache.moistureLow = doc["moisture_low"] | 0;
        m_cache.moistureHigh = doc["moisture_high"] | 0;

#ifdef HARDWARE_PRO
        m_cache.tempLow  = doc["temp_low"] | 0.0f;
        m_cache.tempHigh = doc["temp_high"] | 0.0f;
        m_cache.luxLow   = doc["lux_low"] | 0;
        m_cache.luxHigh  = doc["lux_high"] | 0;
#endif // HARDWARE_PRO

        m_cache.thresholdsPresent = true;
        LOG_SERIAL.println("Parsed response JSON thresholds");
        return true;
    }
    
    return false;
}

// MARK: Initialise
void Sensor::init() {
    // Status indicator
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, HIGH);

    // Initialise debug logging
    LOG_SERIAL.begin(115200);
    LOG_SERIAL.printf("\n--- HydroSensor Initialisation [v%s] ---\n\n", FIRMWARE_VERSION);

    // Attempt to populate cache with RTC cache data
    readCache();

    // Initialise LittleFS for offline backup in case we fail to send sensor data
    if (!LittleFS.begin()) LOG_SERIAL.println("[ERROR] Failed to initialise LittleFS, local backup disabled.");

    // Initialise network via WiFiManager
    WiFiManager wifiManager;
    wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
    wifiManager.setAPCallback(Sensor::configModeCallback);
    wifiManager.setConfigPortalTimeout(180);

    if (!wifiManager.autoConnect(AP_NAME)) {
        LOG_SERIAL.println("[ERROR] Network authentication timeout. Entering deep sleep.");
        enterDeepSleep();
    }

    LOG_SERIAL.printf("Connected to WiFi!\nIP: %s\nMAC: %s.\n\n", WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());

    // Initialise I2C sensors
    m_adsOnline = m_ads.begin();
    if (!m_adsOnline) LOG_SERIAL.println("[ERROR] ADC Bus Offline!");

#ifdef HARDWARE_PRO
    m_shtOnline = m_sht31.begin(0x44);
    if (!m_shtOnline) LOG_SERIAL.println("[ERROR] Temp/Humidity Sensor Offline!");

    m_tslOnline = m_tsl2591.begin();
    if (!m_tslOnline) LOG_SERIAL.println("[ERROR] Light Sensor Offline!");
#endif // HARDWARE_PRO
}

// MARK: OTA Updates
bool Sensor::checkUpdates() {
    ESPhttpUpdate.rebootOnUpdate(false);
    t_httpUpdate_return response = ESPhttpUpdate.update(m_updateClient, UPDATE_HOST, FIRMWARE_VERSION);

    switch (response) {
        case HTTP_UPDATE_OK:
            LOG_SERIAL.println("[OTA] Firmware updated successfully!");
            return true;
        case HTTP_UPDATE_FAILED:
            LOG_SERIAL.printf("[OTA] [ERROR] OTA failed (%d): %s.\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            LOG_SERIAL.println("[OTA] No update required.\n");
            break;
    }
    return false;
}

// MARK: Cache Buffer
void Sensor::appendToBuffer(const String& dataLine) {
    File bufferFile = LittleFS.open(BUFFER_FILE, "a");
    if (!bufferFile) {
        LOG_SERIAL.println("[FS] [ERROR] Failed to open buffer file for writing.");
        return;
    }

    if (bufferFile.println(dataLine)) {
        LOG_SERIAL.println("[FS] Sensor data cached.");
    } else {
        LOG_SERIAL.println("[FS] [ERROR] Failed to write sensor data to flash buffer.");
    }
    bufferFile.close();
}

bool Sensor::dispatchBuffer(HTTPClient& http) {
    if (!LittleFS.exists(BUFFER_FILE)) return true;

    File bufferFile = LittleFS.open(BUFFER_FILE, "r");
    if (!bufferFile) return true;

    String batch = "{ \"time\": \"" + String(m_cache.clock) + "\", \"backlog\": [";
    uint8_t count = 0;

    while (bufferFile.available()) {
        String payload = bufferFile.readStringUntil('\n');
        payload.trim();
        
        if (payload.length() > 0) {
            if (count > 0) batch += ", ";
            batch += payload;
            count++;
        }
    }
    bufferFile.close();
    batch += "]}";

    if (count > 0) {
        LOG_SERIAL.printf("[FS] Sending %u cached telemetry readings to server.", count);
        
        int response = http.POST(batch);
        
        if (response == 201) {
            LOG_SERIAL.println("[FS] Sent cached telemetry buffer to server.");
            LittleFS.remove(BUFFER_FILE);
        } else {
            LOG_SERIAL.printf("[FS] [ERROR] Failed to send cached telemetry buffer. Status: %d.\n", response);
            return false;
        }
    }
    return true;
}

// MARK: Authentication
String Sensor::generateRequestId() const {
    char buffer[17];
    uint32_t r1 = ESP.random();
    uint32_t r2 = ESP.random();
    sprintf(buffer, "%08X%08X", r1, r2);
    return String(buffer);
}

String Sensor::generateApiKey(const String& requestId) const {
    static constexpr uint32_t SECRET = 7291034;
    uint32_t hash = SECRET;
    // Simple bitwise hash
    for (size_t i = 0; i < requestId.length(); i++) {
        hash = (hash << 5) + hash + requestId[i];
    }
    char buffer[16];
    itoa(hash, buffer, 36);
    return String(buffer);
}

// MARK: Dispatch Data
void Sensor::dispatchData() {
    // Base payload
    int charge = ESP.getVcc();

    String payload = "{\"data\": {\"version\": \"" + String(FIRMWARE_VERSION) + "\", \"voltage\": \"" + String(charge) + "\", \"time\": " + String(m_cache.clock);

    // Ingest sensor data
    // No transformation of data is done here, only raw data is sent to server and necessary processing is done there to minimise awake time
    float moisture = m_adsOnline ? m_ads.readADC_SingleEnded(MOISTURE_ADC_CHANNEL) : NAN;
    payload += ", \"moisture\": \"" + String(moisture) + "\"";

#ifdef HARDWARE_PRO
    float temperature = m_shtOnline ? m_sht31.readTemperature() : NAN;
    float humidity = m_shtOnline ? m_sht31.readHumidity() : NAN;
    payload += ", \"temp\": \"" + String(temperature, 2) + "\", \"humidity\": \"" + String(humidity, 2) + "\"";

    float light = m_tslOnline ? m_tsl2591.getLuminosity(TSL2591_VISIBLE) : NAN;
    payload += ", \"lux\": \"" + String(light) + "\"";
#endif // HARDWARE_PRO

    payload += "}}";

    // Generate request ID and API key
    String requestId = generateRequestId();
    String apiKey = generateApiKey(requestId);

    // Dispatch network call
    bool networkSuccessful = false;

    if (DEBUG_MODE) m_secureClient.setInsecure(); // Bypass for development server
    if (m_secureClient.connect(DATA_HOST, 443)) {
        HTTPClient http;
        http.begin(m_secureClient, DATA_HOST);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Request-Id", requestId);
        http.addHeader("X-Api-Key", apiKey);
        http.addHeader("X-Mac-Address", WiFi.macAddress());
        http.setUserAgent(HTTP_USER_AGENT);

        LOG_SERIAL.println("Telemetry update dispatched to central database.");
        int httpResponseCode = http.POST(payload);

        if (httpResponseCode == 201 || httpResponseCode == 200) {
            LOG_SERIAL.println("Telemetry update saved to central database.");
            networkSuccessful = true;

            // Parse thresholds to our memory if server provided them (only provides them if the sensor is linked to an account and setup to a plant)
            WiFiClient* stream = http.getStreamPtr();
            m_cache.thresholdsPresent = false;
            if (stream != nullptr) parseNewThresholds(*stream);

            // Send any backlog as server connection was successful
            if (dispatchBuffer(http)) m_cache.clock = 0;
        } else {
            LOG_SERIAL.printf("[ERROR] Telemetry update failed. Status code: %d.\n", httpResponseCode);
            http.end();
            enterDeepSleep();
        }

        http.end();

        if (httpResponseCode == 200) {
            // 200 status code is returned if successful AND OTA update required, otherwise 201
            if (checkUpdates()) {
                LOG_SERIAL.println("[OTA] Restarting now...\n");
                ESP.restart();
            }
        }
    }

    if (!networkSuccessful) {
        LOG_SERIAL.println("Server unreachable, storing to flash memory.");
        appendToBuffer(payload);
    }

    // Calculate sleep time
    bool reducedSleep = false;

    // Check deltas compared to last readings
    if (m_cache.lastReadingsPresent) {
        // Trigger on 3% change of moisture reading-over-reading
        if (m_adsOnline && (isnan(m_cache.lastMoisture) || (fabs(moisture - m_cache.lastMoisture) > 528.0f))) reducedSleep = true;
#ifdef HARDWARE_PRO
        // Trigger on 2 degrees celcius change of temperature reading-over-reading
        if (m_shtOnline && (isnan(m_cache.lastTemp) || (fabs(temperature - m_cache.lastTemp) > 2.0f))) reducedSleep = true;
        // Trigger on 5000 lux change reading-over-reading
        if (m_tslOnline && (isnan(m_cache.lastLux) || (fabs(light - m_cache.lastLux) > 5000.0f))) reducedSleep = true;
#endif
    }

    // Check readings compared to thresholds
    if (m_cache.thresholdsPresent) {
        if (m_adsOnline) {
            float mBuffer = (m_cache.moistureHigh - m_cache.moistureLow) * 0.10f;
            if (moisture <= (m_cache.moistureLow + mBuffer) || moisture >= (m_cache.moistureHigh - mBuffer)) reducedSleep = true;
        }
#ifdef HARDWARE_PRO
        if (m_shtOnline) {
            float tBuffer = (m_cache.tempHigh - m_cache.tempLow) * 0.10f;
            if (temperature <= (m_cache.tempLow + tBuffer) || temperature >= (m_cache.tempHigh - tBuffer)) reducedSleep = true;
        }
        if (m_tslOnline) {
            float lBuffer = (m_cache.luxHigh - m_cache.luxLow) * 0.10f;
            if (light <= (m_cache.luxLow + lBuffer) || light >= (m_cache.luxHigh - lBuffer)) reducedSleep = true;
        }
#endif
    }

    // Save state context parameters for the next wakeup assessment phase
    m_cache.lastMoisture = moisture;
#ifdef HARDWARE_PRO
    m_cache.lastTemp = temperature;
    m_cache.lastLux = light;
#endif
    m_cache.lastReadingsPresent = true;
    writeCache(); 

    if (reducedSleep) {
        m_sleepInterval = BASE_INTERVAL / 3;
        LOG_SERIAL.println("Volatile environmental detected, reducing sleep time.");
    } else {
        m_sleepInterval = BASE_INTERVAL;
    }
    
    enterDeepSleep();
}

void Sensor::enterDeepSleep() {
    digitalWrite(LED_GPIO, LOW);
    LOG_SERIAL.println("Entering deep sleep.\n");
    ESP.deepSleep(m_sleepInterval);
}

// MARK: AP Config Mode
void IRAM_ATTR Sensor::onTimerISR() {
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO)); // Blink
    timer1_write(5e6);
}

void Sensor::configModeCallback(WiFiManager* wifiManager) {
    (void)wifiManager;
    LOG_SERIAL.printf("AP Captive Portal Active. IP: %s.\n", WiFi.softAPIP().toString().c_str());
    timer1_attachInterrupt(Sensor::onTimerISR); // Blink status indicator
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(5e6);
}
