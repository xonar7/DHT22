/*
 * TTGO LoRa32-OLED + DHT22 Temperature & Humidity Telemetry
 * OPTIMIZADO PARA TRANSMISIÓN EXACTA CADA 10 SEGUNDOS
 * CON OPTIMIZACIÓN DE ENERGÍA DESDE POWERBANK
 * Aplicación: dht22ap - Dispositivo: dht22ed
 * 
 * CONVERSIÓN v1.0 - DHT22 SENSOR:
 * - Soporte para temperatura Y humedad
 * - Identificación del tipo de sensor
 * - Payload de 6 bytes con status del sensor
 * - Display OLED mostrando temperatura y humedad
 * - Manejo robusto de errores
 * - Watchdog para recuperación automática
 */

#include <Arduino.h>

// -------------------- CONFIGURACIÓN LMIC (CRÍTICO: ANTES DE INCLUDES) --------------------
#define CFG_sx1276_radio 1
#define CFG_us915 1

// -------------------- LIBRERÍAS NECESARIAS --------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <esp_task_wdt.h>

// -------------------- CONFIGURACIÓN HARDWARE --------------------
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define LED_PIN 25

// -------------------- CONFIGURACIÓN DHT22 --------------------
#define DHTPIN 21           // GPIO para DHT22
#define DHTTYPE DHT22       // Tipo de sensor DHT

// Watchdog timeout (30 segundos)
#define WDT_TIMEOUT 30

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
DHT dht(DHTPIN, DHTTYPE);

// -------------------- CONFIGURACIÓN LORAWAN --------------------
// IMPORTANTE: Estos valores deben coincidir con TTN
// Credenciales de dht22ap / dht22ed
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM DEVEUI[8] = { 0x3C, 0x71, 0xBF, 0xFE, 0xFF, 0xF1, 0x81, 0xC4 };
static const u1_t PROGMEM APPKEY[16] = { 
    0x91, 0xDC, 0x50, 0x94, 0xEA, 0x51, 0xC1, 0xF4, 
    0x5A, 0x1F, 0xAF, 0x97, 0x44, 0x9B, 0x9C, 0x13 
};

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

// -------------------- VARIABLES GLOBALES OPTIMIZADAS --------------------
static uint8_t mydata[6]; // 6 bytes: [Temp_MSB, Temp_LSB, Hum_MSB, Hum_LSB, Status, SensorType]
static osjob_t sendjob;

// TIMING PRECISO
const unsigned long TX_INTERVAL_MS = 10000; // 10 segundos exactos
unsigned long lastTransmissionTime = 0;
bool forceTransmission = false;

// Variables de estado - DHT22
float currentTemp = 0.0;
float currentHum = 0.0;
bool sensorValid = false;
int tx_count = 0;
String network_status = "Iniciando...";
bool joined = false;
uint8_t last_event = 0;
int consecutiveSensorErrors = 0;

// Variables para monitoreo de timing
unsigned long actualIntervals[10] = {0};
int intervalIndex = 0;

// Variables para ahorro de energía
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 5000; // Actualizar display cada 5s
bool displayOn = true;

// -------------------- FUNCIONES DE GESTIÓN DE ENERGÍA --------------------
void displaySleep() {
    if (displayOn) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        displayOn = false;
    }
}

void displayWake() {
    if (!displayOn) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        displayOn = true;
    }
}

// -------------------- FUNCIONES DE TIMING PRECISO --------------------
void calculateTiming() {
    if (lastTransmissionTime > 0) {
        unsigned long actualInterval = millis() - lastTransmissionTime;
        actualIntervals[intervalIndex] = actualInterval;
        intervalIndex = (intervalIndex + 1) % 10;
        
        Serial.printf("⏱️ Intervalo real: %lu ms (objetivo: %lu ms)\n", actualInterval, TX_INTERVAL_MS);
        
        // Calcular promedio de últimos 10 intervalos
        unsigned long sum = 0;
        int count = 0;
        for (int i = 0; i < 10; i++) {
            if (actualIntervals[i] > 0) {
                sum += actualIntervals[i];
                count++;
            }
        }
        
        if (count > 0) {
            Serial.printf("📊 Promedio últimos %d: %.1f s\n", count, sum / (count * 1000.0));
        }
    }
}

bool isTimeToTransmit() {
    unsigned long currentTime = millis();
    
    if (lastTransmissionTime == 0 || forceTransmission) {
        return true;
    }
    
    return (currentTime - lastTransmissionTime >= TX_INTERVAL_MS);
}

unsigned long getTimeUntilNext() {
    if (lastTransmissionTime == 0) return 0;
    
    unsigned long elapsed = millis() - lastTransmissionTime;
    if (elapsed >= TX_INTERVAL_MS) return 0;
    
    return (TX_INTERVAL_MS - elapsed) / 1000; // En segundos
}

// -------------------- FUNCIONES DE SENSOR DHT22 --------------------
void initializeSensor() {
    dht.begin();
    Serial.println("\n🌡️ Sensor DHT22 inicializado");
    Serial.printf("   Pin GPIO: %d\n", DHTPIN);
    Serial.println("   Tipo: DHT22 (AM2302)");
    Serial.println("   Rango Temp: -40°C a +80°C");
    Serial.println("   Rango Hum: 0% a 100%");
    
    // Lectura inicial de prueba
    delay(2000); // DHT22 necesita 2s después de encendido
    
    float testTemp = dht.readTemperature();
    float testHum = dht.readHumidity();
    
    if (!isnan(testTemp) && !isnan(testHum)) {
        Serial.println("   ✓ Sensor respondiendo correctamente");
        Serial.printf("   Lectura inicial: %.1f°C, %.1f%%\n", testTemp, testHum);
    } else {
        Serial.println("   ⚠️ ADVERTENCIA: Error en lectura inicial");
    }
}

void readSensorData() {
    // Leer temperatura y humedad
    currentTemp = dht.readTemperature();
    currentHum = dht.readHumidity();
    
    // Validar lecturas
    if (isnan(currentTemp) || isnan(currentHum)) {
        consecutiveSensorErrors++;
        Serial.printf("❌ DHT22: Error lectura (errores consecutivos: %d)\n", consecutiveSensorErrors);
        
        sensorValid = false;
        currentTemp = -999.0;
        currentHum = -999.0;
        
        // Reintentar inicialización si hay demasiados errores
        if (consecutiveSensorErrors > 5) {
            Serial.println("🔄 DHT22: Reintentando inicialización...");
            dht.begin();
            consecutiveSensorErrors = 0;
        }
        return;
    }
    
    // Validar rangos del DHT22
    // Temperatura: -40°C a +80°C (típicamente)
    // Humedad: 0% a 100%
    if (currentTemp < -40.0 || currentTemp > 80.0 || 
        currentHum < 0.0 || currentHum > 100.0) {
        consecutiveSensorErrors++;
        Serial.printf("❌ DHT22: Valores fuera de rango (T:%.1f°C, H:%.1f%%)\n", currentTemp, currentHum);
        sensorValid = false;
        return;
    }
    
    // Lectura válida
    consecutiveSensorErrors = 0;
    sensorValid = true;
    
    Serial.printf("🌡️ DHT22: %.1f°C, %.1f%% RH (OK)\n", currentTemp, currentHum);
}

void prepareTempHumData() {
    // Leer datos del sensor
    readSensorData();
    
    // Si hay errores, usar valores por defecto
    float tempToSend = sensorValid ? currentTemp : -999.0;
    float humToSend = sensorValid ? currentHum : -999.0;
    
    // Convertir a formato de transmisión (int16_t × 100 y uint16_t × 100)
    int16_t tempInt = (int16_t)(tempToSend * 100);
    uint16_t humInt = (uint16_t)(humToSend * 100);
    
    // Construir payload de 6 bytes
    mydata[0] = (tempInt >> 8) & 0xFF;  // Temp MSB
    mydata[1] = tempInt & 0xFF;         // Temp LSB
    mydata[2] = (humInt >> 8) & 0xFF;   // Hum MSB
    mydata[3] = humInt & 0xFF;          // Hum LSB
    
    // Status byte: bit 0 = sensor válido
    mydata[4] = sensorValid ? 0x01 : 0x00;
    
    // Identificador: 0x22 para DHT22
    mydata[5] = 0x22;
    
    Serial.printf("📦 Payload: [%02X %02X %02X %02X %02X %02X]\n", 
                  mydata[0], mydata[1], mydata[2], mydata[3], mydata[4], mydata[5]);
}

// -------------------- FUNCIONES DE PANTALLA - DHT22 --------------------
void updateTempHumDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    
    // ========== TEMPERATURA ==========
    // Etiqueta pequeña
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("TEMPERATURA:");
    
    // Temperatura GRANDE
    display.setTextSize(2);
    display.setCursor(0, 12);
    if (sensorValid) {
        display.printf("%.1f", currentTemp);
        display.setTextSize(1);
        display.print("o");
        display.setTextSize(2);
        display.print("C");
    } else {
        display.setTextSize(2);
        display.print("ERROR");
    }
    
    // ========== HUMEDAD ==========
    // Etiqueta pequeña
    display.setTextSize(1);
    display.setCursor(0, 38);
    display.print("HUMEDAD:");
    
    // Humedad GRANDE
    display.setTextSize(2);
    display.setCursor(0, 50);
    if (sensorValid) {
        display.printf("%.1f", currentHum);
        display.setTextSize(2);
        display.print("%");
    } else {
        display.setTextSize(2);
        display.print("ERROR");
    }
    
    display.display();
}

void updateInfoDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.println("=== STATS DHT22 ===");
    
    display.setCursor(0, 12);
    display.printf("TX Total: %d", tx_count);
    
    display.setCursor(0, 22);
    display.printf("Next TX: %lus", getTimeUntilNext());
    
    display.setCursor(0, 32);
    display.printf("Pin: %d | DHT22", DHTPIN);
    
    display.setCursor(0, 42);
    display.printf("Sensor:%s", sensorValid ? "OK" : "ER");
    
    display.setCursor(0, 52);
    display.printf("Ev:%d Err:%d", last_event, consecutiveSensorErrors);
    
    display.display();
}

void updateDisplay() {
    displayWake();
    updateTempHumDisplay(); // Solo mostrar temperatura y humedad
}

// -------------------- TRANSMISIÓN OPTIMIZADA --------------------
void do_send(osjob_t* j) {
    esp_task_wdt_reset();
    
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("⏸️ TX ocupado - reintento en 500ms");
        os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(500), do_send);
        return;
    }
    
    if (!joined) {
        Serial.println("⏸️ No conectado - reintento en 1s");
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
        return;
    }
    
    if (!forceTransmission && !isTimeToTransmit()) {
        unsigned long msUntilNext = TX_INTERVAL_MS - (millis() - lastTransmissionTime);
        Serial.printf("⏳ Esperando %lu ms para timing exacto\n", msUntilNext);
        os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(msUntilNext), do_send);
        return;
    }
    
    calculateTiming();
    
    digitalWrite(LED_PIN, HIGH);
    
    prepareTempHumData();
    
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    
    lastTransmissionTime = millis();
    forceTransmission = false;
    tx_count++;
    
    Serial.printf("📡 TX #%d enviado\n", tx_count);
    
    delay(30);
    digitalWrite(LED_PIN, LOW);
    
    updateDisplay();
}

// -------------------- EVENTOS LMIC OPTIMIZADOS --------------------
void onEvent(ev_t ev) {
    last_event = ev;
    esp_task_wdt_reset();
    
    switch (ev) {
        case EV_JOINING:
            Serial.println("🔗 Uniendo a TTN dht22ap...");
            network_status = "Joining...";
            break;
            
        case EV_JOINED:
            Serial.println("✅ ¡CONECTADO A dht22ap!");
            network_status = "Connected!";
            joined = true;
            
            // Configuración de canales US915
            Serial.println("🔧 Configurando canales US915...");
            for (int i = 0; i < 72; i++) {
                if (i != 8 && i != 16 && i != 24 && i != 32 && 
                    i != 40 && i != 48 && i != 56 && i != 64) {
                    LMIC_disableChannel(i);
                }
            }
            
            LMIC_setLinkCheckMode(0);
            LMIC_setDrTxpow(DR_SF7, 14);
            LMIC_setAdrMode(0);
            LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
            
            Serial.println("⚙️ Optimizaciones aplicadas: ADR OFF, SF7 fijo");
            
            forceTransmission = true;
            lastTransmissionTime = 0;
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
            
        case EV_JOIN_FAILED:
            Serial.println("❌ Error de conexión");
            network_status = "Join Failed";
            joined = false;
            break;
            
        case EV_TXCOMPLETE:
            {
                Serial.println("✓ TX completado");
                network_status = "TX OK";
                
                if (LMIC.txrxFlags & TXRX_ACK) {
                    Serial.println("📨 ACK recibido");
                }
                
                if (LMIC.dataLen) {
                    Serial.printf("📥 Downlink: %d bytes\n", LMIC.dataLen);
                }
                
                unsigned long msUntilNext = TX_INTERVAL_MS;
                os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(msUntilNext), do_send);
                
                Serial.printf("⏰ Próxima TX programada en %lu ms\n", msUntilNext);
            }
            break;
            
        case EV_RESET:
            Serial.println("🔄 LMIC Reset");
            network_status = "Reset";
            break;
            
        default:
            Serial.printf("ℹ️ Evento: %d\n", (unsigned) ev);
            break;
    }
    
    updateDisplay();
}

// -------------------- SETUP OPTIMIZADO --------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=================================");
    Serial.println(" LoRaWAN DHT22 v1.0");
    Serial.println(" TEMPERATURA + HUMEDAD");
    Serial.println(" Transmisión: 10.0s exactos");
    Serial.println(" ADR OFF, SF7 fijo, Clock 2%");
    Serial.println("=================================\n");
    
    // Configurar watchdog
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    Serial.println("🐕 Watchdog activado (30s timeout)");
    
    // Configurar LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Configurar DHT22
    initializeSensor();
    
    // Configurar OLED
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(5);
    digitalWrite(OLED_RST, HIGH);
    
    Wire.begin(OLED_SDA, OLED_SCL);
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
        Serial.println("❌ Error OLED");
        for(;;);
    }
    
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("DHT22 v1.0");
    display.setCursor(0, 16);
    display.println("LoRaWAN Temp+Hum");
    display.setCursor(0, 32);
    display.println("Cada 10.0s exactos");
    display.setCursor(0, 48);
    display.println("Conectando TTN...");
    display.display();
    
    // Test LED
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    
    // Configuración LMIC
    os_init();
    LMIC_reset();
    LMIC_selectSubBand(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(0);
    
    Serial.println("🚀 Iniciando conexión optimizada...");
    LMIC_startJoining();
    
    do_send(&sendjob);
}

// -------------------- LOOP OPTIMIZADO --------------------
void loop() {
    esp_task_wdt_reset();
    os_runloop_once();
    yield();
}