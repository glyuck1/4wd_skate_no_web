#include <Arduino.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

// ===== UART ДЛЯ VX1 =====
#define VX1_RX_PIN 16
#define VX1_TX_PIN 17
#define VX1_BAUDRATE 115200

// ===== UART ДЛЯ ODRIVE =====
#define ODRIVE_RX_PIN 25
#define ODRIVE_TX_PIN 26
#define ODRIVE_BAUDRATE 115200 // УСТАНОВИ ТО ЖЕ, ЧТО В ODRIVE (9600 или 115200)

// Безопасность
#define VX1_TIMEOUT_MS 500
#define MAX_VELOCITY 5.0

// ===== UART ОБЪЕКТЫ =====
HardwareSerial VX1Serial(1);
HardwareSerial ODriveSerial(2);

// ODrive объект
ODriveArduino odrive(ODriveSerial);

// ===== СТРУКТУРЫ ДАННЫХ =====
struct VX1Data {
    float throttle = 0.0;
    unsigned long last_update = 0;
    bool signal_ok = false;
    uint32_t packet_count = 0;
    uint32_t error_count = 0;
};

struct ODriveStatus {
    float velocity[4] = {0, 0, 0, 0};
    float bus_voltage = 0.0;
    bool connected = false;
    unsigned long last_update = 0;
};

VX1Data vx1;
ODriveStatus odrive_status;
bool emergency_stop_active = false;
bool motors_calibrated = false;

// ===== ПРОТОТИПЫ =====
void setupVX1();
void setupODrive();
void parseVX1Packet();
void updateODrive();
void emergencyStop();
void checkODriveStatus();
void calibrateMotors();
uint16_t crc16(const uint8_t* data, uint16_t len);

// ===== CRC16 =====
uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// ===== SETUP VX1 =====
void setupVX1() {
    VX1Serial.begin(VX1_BAUDRATE, SERIAL_8N1, VX1_RX_PIN, VX1_TX_PIN);
    Serial.println("VX1 UART initialized");
}

// ===== Вспомогательная функция чтения Vbus для Pro =====
float getVbusPro() {
    ODriveSerial.println("r vbus_voltage");
    unsigned long timeout = millis() + 150;
    String response = "";
    while (millis() < timeout) {
        if (ODriveSerial.available()) {
            char c = ODriveSerial.read();
            if (c == '\n') break;
            response += c;
        }
    }
    return response.toFloat();
}

// ===== SETUP ODRIVE =====
void setupODrive() {
    // Внимание: пины 25/26 для Serial2 на ESP32
    ODriveSerial.begin(ODRIVE_BAUDRATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    
    Serial.println("Checking ODrive Pro connection...");
    delay(1000);
    
    // В Pro-версии стандартный GetParameter часто тупит, используем прямой ASCII запрос
    odrive_status.bus_voltage = getVbusPro();
    
    if (odrive_status.bus_voltage > 5.0) {
        Serial.print("   Vbus: ");
        Serial.print(odrive_status.bus_voltage);
        Serial.println(" V");
        odrive_status.connected = true;
        Serial.println("ODrive connected!");
    } else {
        Serial.println("No response from ODrive. Trying fallback...");
        // Попытка №2
        odrive_status.bus_voltage = getVbusPro();
        if (odrive_status.bus_voltage > 5.0) {
            odrive_status.connected = true;
        } else {
            odrive_status.connected = false;
        }
    }
}

// ===== КАЛИБРОВКА МОТОРОВ =====
void calibrateMotors() {
    if (!odrive_status.connected) return;
    
    Serial.println("Starting calibration sequence...");
    // Для ODrive Pro: 3 — FULL_CALIBRATION
    ODriveSerial.println("w axis0.requested_state 3");
    delay(500);
    ODriveSerial.println("w axis1.requested_state 3");
    
    Serial.println("Waiting for calibration (approx 20s)...");
    delay(20000); 
    
    motors_calibrated = true;
    Serial.println("Calibration finished");
}

// ===== ПАРСИНГ VX1 =====
void parseVX1Packet() {
    static uint8_t buffer[256];
    static uint8_t buf_idx = 0;
    static bool in_packet = false;
    
    while (VX1Serial.available()) {
        uint8_t byte = VX1Serial.read();
        if (!in_packet) {
            if (byte == 0x02 || byte == 0x03) {
                buffer[0] = byte;
                buf_idx = 1;
                in_packet = true;
            }
            continue;
        }
        if (buf_idx < sizeof(buffer)) {
            buffer[buf_idx++] = byte;
        } else {
            in_packet = false;
            buf_idx = 0;
            vx1.error_count++;
            continue;
        }
        if (byte == 0x03 && buf_idx > 3) {
            in_packet = false;
            if (buf_idx < 6) { vx1.error_count++; buf_idx = 0; continue; }
            uint8_t payload_len = buffer[1];
            uint16_t received_crc = (buffer[buf_idx - 3] << 8) | buffer[buf_idx - 2];
            uint16_t calculated_crc = crc16(&buffer[2], payload_len);
            if (received_crc != calculated_crc) { vx1.error_count++; buf_idx = 0; continue; }
            vx1.packet_count++;
            vx1.last_update = millis();
            vx1.signal_ok = true;
            uint8_t command = buffer[2];
            if (command == 0x23 && payload_len >= 3) {
                uint8_t stick_value = buffer[4];
                float normalized = (stick_value - 127.0f) / 127.0f;
                vx1.throttle = constrain(normalized, -1.0, 1.0);
            }
            buf_idx = 0;
        }
    }
    if (millis() - vx1.last_update > VX1_TIMEOUT_MS) {
        if (vx1.signal_ok) Serial.println("VX1 SIGNAL LOST!");
        vx1.signal_ok = false;
        vx1.throttle = 0.0;
    }
}

// ===== АВАРИЙНАЯ ОСТАНОВКА =====
void emergencyStop() {
    emergency_stop_active = true;
    // v <axis> <velocity> <torque_ff>
    ODriveSerial.println("v 0 0 0");
    ODriveSerial.println("v 1 0 0");
    Serial.println("EMERGENCY STOP!");
}

// ===== ПРОВЕРКА СТАТУСА ODRIVE =====
void checkODriveStatus() {
    static unsigned long last_check = 0;
    if (millis() - last_check < 2000) return; // Раз в 2 сек достаточно
    last_check = millis();
    if (!odrive_status.connected) return;

    // Прямой опрос скоростей (библиотечные геттеры могут вешать ESP)
    ODriveSerial.println("r axis0.encoder.vel_estimate");
    delay(10);
    if(ODriveSerial.available()) odrive_status.velocity[0] = ODriveSerial.readStringUntil('\n').toFloat();
    
    ODriveSerial.println("r axis1.encoder.vel_estimate");
    delay(10);
    if(ODriveSerial.available()) odrive_status.velocity[1] = ODriveSerial.readStringUntil('\n').toFloat();
}

// ===== ОБНОВЛЕНИЕ ODRIVE =====
void updateODrive() {
    if (!odrive_status.connected) return;
    
    if (!vx1.signal_ok) {
        if (!emergency_stop_active) emergencyStop();
        return;
    }
    
    if (emergency_stop_active) {
        emergency_stop_active = false;
        Serial.println("Restoring control...");
        ODriveSerial.println("w axis0.requested_state 8");
        ODriveSerial.println("w axis1.requested_state 8");
    }
    
    float target_velocity = vx1.throttle * MAX_VELOCITY;
    
    // Используем ASCII формат: v <axis> <vel> <torque_ff>
    ODriveSerial.print("v 0 ");
    ODriveSerial.print(target_velocity, 3);
    ODriveSerial.println(" 0");
    
    ODriveSerial.print("v 1 ");
    ODriveSerial.print(target_velocity, 3);
    ODriveSerial.println(" 0");
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    setupVX1();
    setupODrive();
    
    if (odrive_status.connected) {
        Serial.println("Send 'y' to calibrate, or any other key to skip...");
        unsigned long start = millis();
        bool do_calib = false;
        while (millis() - start < 5000) {
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 'y' || c == 'Y') { do_calib = true; break; }
            }
        }
        
        if (do_calib) calibrateMotors();
        else {
            motors_calibrated = true;
            //requested_state 8 - CLOSED_LOOP_CONTROL
            ODriveSerial.println("w axis0.requested_state 8");
            ODriveSerial.println("w axis1.requested_state 8");
        }
    }
}

// ===== LOOP =====
void loop() {
    parseVX1Packet();
    updateODrive();
    checkODriveStatus();

    // Отладка раз в 5 сек (как в твоем коде)
    static unsigned long last_status = 0;
    if (millis() - last_status > 5000) {
        last_status = millis();
        Serial.print("VX1: "); Serial.print(vx1.signal_ok ? "OK " : "LOST ");
        Serial.print("ODrv: "); Serial.print(odrive_status.connected ? "ON " : "OFF ");
        Serial.print("Thr: "); Serial.println(vx1.throttle, 2);
    }
    
    delay(20);
}