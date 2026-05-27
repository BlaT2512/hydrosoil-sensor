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

// MARK: RTC Cache
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
            // Cold boot / battery swap - initialise with base data
            m_cache.clock = 0;
            m_cache.lastSleep = 0;
            m_cache.failedRequests = 0;
            LittleFS.remove(BUFFER_FILE); // Backlog is dropped as time of requests cannot be determined (RTC cache wiped)
        } else {
            m_cache.clock += (m_cache.lastSleep / 1000000); // Accumulate time just slept
            LOG_SERIAL.println("Using RTC cache.");
        }
    }
}

void Sensor::writeCache() {
    m_cache.lastSleep = m_sleepInterval;
    m_cache.checksum = calculateChecksum(((uint8_t*)&m_cache) + 4, sizeof(m_cache) - 4);
    ESP.rtcUserMemoryWrite(64, (uint32_t*)&m_cache, sizeof(m_cache));
}

// MARK: Initialise
void Sensor::init() {
    // Status indicator
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, HIGH);

    // Initialise debug logging
    LOG_SERIAL.begin(115200);
    LOG_SERIAL.printf("\n--- HydroSensor Initialisation [v%s] ---\n\n", FIRMWARE_VERSION);

    // Initialise LittleFS for offline backup in case we fail to send sensor data
    if (!LittleFS.begin()) LOG_SERIAL.println("[ERROR] Failed to initialise LittleFS, local backup disabled.");

    // Attempt to populate cache with RTC cache data
    readCache();

    // Initialise network via WiFiManager
    WiFiManager wifiManager;
    wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
    wifiManager.setAPCallback(Sensor::configModeCallback);
    wifiManager.setConfigPortalTimeout(180);

    if (!wifiManager.autoConnect(AP_NAME)) {
        LOG_SERIAL.println("[ERROR] Network authentication timeout. Entering deep sleep.");

        // Gather sensor data and add to local buffer
        String reading = collectSensorData();
        appendToBuffer(reading);

        m_cache.failedRequests++;
        m_sleepInterval = (m_cache.failedRequests <= 3) ? (BASE_INTERVAL / 5) : BASE_INTERVAL; // Backoff retry time from 1 minute to usual 5 after 3 failures
        writeCache();
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

String Sensor::readBuffer() {
    if (!LittleFS.exists(BUFFER_FILE)) return "";

    File bufferFile = LittleFS.open(BUFFER_FILE, "r");
    if (!bufferFile) return "";

    String payload = "";
    uint8_t count = 0;

    while (bufferFile.available()) {
        String line = bufferFile.readStringUntil('\n');
        line.trim();

        if (line.length() > 0) {
            if (count == 0) payload += "[";
            else payload += ", ";
            payload += line;
            count++;
        }
    }
    bufferFile.close();
    if (count > 0) payload += "]";
    return payload;
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

// MARK: Collect Data
String Sensor::collectSensorData() {
    int charge = ESP.getVcc();
    
    // Base payload
    String payload = "{\"version\": \"" + String(FIRMWARE_VERSION) + "\", \"voltage\": \"" + String(charge) +  "\", \"time\": " + String(m_cache.clock);

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
#endif

    payload += "}";
    return payload;
}

// MARK: Dispatch Data
void Sensor::dispatchData() {
    // Collect sensor data
    String payload = collectSensorData();

    // Check for backlog of failed requests, and attach if necessary
    String fullPayload = "{\"data\": " + payload;

    String backlog = readBuffer();
    if (backlog != "") fullPayload += ", \"backlog\": " + backlog;

    fullPayload += "}";

    // Generate request ID and API key
    String requestId = generateRequestId();
    String apiKey = generateApiKey(requestId);

    // Dispatch network call
    bool networkSuccessful = false;
    m_sleepInterval = BASE_INTERVAL;

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
        int httpResponseCode = http.POST(fullPayload);

        if (httpResponseCode == 201 || httpResponseCode == 200) {
            LOG_SERIAL.println("Telemetry update saved to central database.");
            networkSuccessful = true;

            // Reset clock and failed requests on success
            m_cache.clock = 0;
            m_cache.failedRequests = 0;
            if (backlog != "") LittleFS.remove(BUFFER_FILE);

            // Read sleep instructions from response
            String response = http.getString();
            response.trim();

            if (response.length() > 0) {
                uint32_t sleepTime = response.toInt();
                if (sleepTime > 0) m_sleepInterval = sleepTime;
            }
        } else {
            LOG_SERIAL.printf("[ERROR] Telemetry update failed. Status code: %d.\n", httpResponseCode);
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

        m_cache.failedRequests++;
        m_sleepInterval = (m_cache.failedRequests <= 3) ? (BASE_INTERVAL / 5) : BASE_INTERVAL; // Backoff retry time from 1 minute to usual 5 after 3 failures
    }

    writeCache();
    enterDeepSleep();
}

void Sensor::enterDeepSleep() {
    digitalWrite(LED_GPIO, LOW);
    LOG_SERIAL.printf("Entering deep sleep for %u.\n\n", m_sleepInterval);
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
