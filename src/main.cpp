#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/twai.h>
#include <Preferences.h>

// ===== UART ДЛЯ VX1 =====
#define VX1_RX_PIN 16
#define VX1_TX_PIN 17
#define VX1_BAUDRATE 115200

// ===== CAN ДЛЯ ODRIVE =====
#define CAN_TX_PIN 21
#define CAN_RX_PIN 22

// ===== PWM OUT =====
#define PIN_PWM_OUT1 25   // LEDC ch0 — передний свет
#define PIN_PWM_OUT2 26   // LEDC ch1 — задний тормоз

// ===== LEDC каналы =====
#define LEDC_CH_FRONT  0
#define LEDC_CH_REAR   1
#define LEDC_FREQ 5000    
#define LEDC_RES  8 

// ===== ODrive Node IDs =====
#define NUM_AXES 4
#define AXIS_FL 0
#define AXIS_FR 1
#define AXIS_RL 2
#define AXIS_RR 3

// ===== CAN Command IDs =====
#define CMD_HEARTBEAT        0x01
#define CMD_GET_MOTOR_ERR    0x03
#define CMD_SET_AXIS_STATE   0x07
#define CMD_GET_ENCODER_EST  0x09
#define CMD_SET_INPUT_TRQ    0x0E
#define CMD_GET_IQ           0x14
#define CMD_GET_TEMP         0x15
#define CMD_GET_BUS_VI       0x17
#define CMD_CLEAR_ERRORS     0x18

// ===== ODrive States =====
#define AXIS_STATE_IDLE                1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

// ===== Настройки Батареи (Limp Mode) =====
#define BATT_CELLS          10      // Укажи количество ячеек (10S, 12S, 14S)
#define BATT_V_MAX          (4.2f * BATT_CELLS)
#define BATT_V_LIMP         (3.5f * BATT_CELLS) // Начало урезания тяги
#define BATT_V_MIN          (3.2f * BATT_CELLS) // Полный разряд

// ===== Параметры системы =====
#define VX1_TIMEOUT_MS      300
#define DEADZONE            0.05f

#define MAX_TORQUE_FWD      2.0f     
#define MAX_TORQUE_BRAKE    1.0f     
#define TORQUE_RAMP_RATE    2.0f    
#define EMERGENCY_RAMP_RATE 0.5f     // Скорость остановки при обрыве связи

#define BRAKE_VEL_THRESH    0.2f     // Порог остановки (r/s)

#define FRONT_LIGHT_DEFAULT 255
#define BRAKE_LIGHT_BRIGHT  255
#define BRAKE_ON_MS   300
#define BRAKE_OFF_MS  100

// =========================================================
Preferences prefs;
HardwareSerial VX1Serial(2);

// =========================================================
struct AxisTelemetry {
    float vel_estimate = 0.0f;
    float iq_measured  = 0.0f;
    float bus_voltage  = 0.0f;
    float temperature  = 0.0f;
    uint32_t axis_error  = 0;
    uint8_t  axis_state  = 0;
    bool     connected   = false;
    unsigned long last_heartbeat = 0;
};

struct VX1Data {
    float throttle     = 0.0f;   // -1..+1
    float raw_adc      = 0;      
    unsigned long last_valid = 0;
    bool signal_ok     = false;
    bool initial_check_done = false; // Для Stuck Throttle
    
    // Калибровка из NVS
    int cal_center     = 128;
    int cal_min        = 0;     
    int cal_max        = 252;  
    bool calibrated    = false;
};

enum SystemState {
    STATE_DISARMED,
    STATE_ARMED,
    STATE_RAMPING_DOWN,  // Плавная остановка (VX1 lost)
    STATE_FATAL_ERROR    // Заклинил пульт
};

struct SystemStatus {
    SystemState state         = STATE_DISARMED;
    bool neutral_locked       = true;
    float current_torque      = 0.0f;  
    float target_torque       = 0.0f;
    bool can_initialized      = false;
    unsigned long last_can_tx = 0;
};

enum ArmPhase { ARM_IDLE, ARM_UP, ARM_DOWN };

struct GestureControl {
    ArmPhase arm_phase = ARM_IDLE;
    unsigned long arm_timer = 0;
    
    bool reverse_held = false;
    unsigned long reverse_timer = 0;
    bool disarm_triggered = false;
    bool clear_triggered = false;
};

struct OutputConfig {
    uint8_t front_brightness = FRONT_LIGHT_DEFAULT;
    bool    front_enabled    = false;
    bool    rear_enabled     = false; 
};

struct BrakeLight {
    bool braking        = false;
    unsigned long phase_start = 0;
    bool light_on       = false;
};

AxisTelemetry  axis[NUM_AXES];
VX1Data        vx1;
SystemStatus   sys;
GestureControl gesture;
OutputConfig   outCfg;
BrakeLight     brakeLight;

// =========================================================
// Прототипы
// =========================================================
void setupPins();
void setupVX1();
void setupCAN();
void loadCalibration();
void parseVX1Packet();
void processGestures();
void updateSystem();
void readCANMessages();
void updateOutputs();
bool sendCAN(uint8_t axis_id, uint8_t cmd_id, const uint8_t* data, uint8_t len);
void setAxisState(uint8_t axis_id, uint8_t state);
bool setTorque(uint8_t axis_id, float torque);
void clearErrors(uint8_t axis_id);
void clearAllErrors();
void armSystem();
void disarmSystem();
uint16_t crc16(const uint8_t* data, uint16_t len);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);

// =========================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32 4WD SKATE CONTROLLER (EV PRO) ===\n");

    setupPins();
    loadCalibration();
    setupVX1();
    setupCAN();
    
    Serial.println("System Ready.");
}

void loop() {
    parseVX1Packet();
    readCANMessages();
    processGestures();
    updateSystem();
    delay(5);
}

// =========================================================
void setupPins() {
    ledcSetup(LEDC_CH_FRONT, LEDC_FREQ, LEDC_RES);
    ledcSetup(LEDC_CH_REAR,  LEDC_FREQ, LEDC_RES);
    ledcAttachPin(PIN_PWM_OUT1, LEDC_CH_FRONT);
    ledcAttachPin(PIN_PWM_OUT2, LEDC_CH_REAR);
    ledcWrite(LEDC_CH_FRONT, 0);
    ledcWrite(LEDC_CH_REAR,  0);
}

void setupVX1() {
    VX1Serial.begin(VX1_BAUDRATE, SERIAL_8N1, VX1_RX_PIN, VX1_TX_PIN);
}

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 20;

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
        sys.can_initialized = true;
        Serial.println("CAN initialized 500kbps");
        delay(300); 
        for (int i = 0; i < NUM_AXES; i++) { setAxisState(i, AXIS_STATE_IDLE); delay(10); }
    }
}

void loadCalibration() {
    prefs.begin("skate", true);
    vx1.cal_center  = prefs.getInt("cal_ctr",  128);
    vx1.cal_min     = prefs.getInt("cal_min",  0);
    vx1.cal_max     = prefs.getInt("cal_max",  252);
    vx1.calibrated  = prefs.getBool("cal_ok",  false);
    prefs.end();
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =========================================================
// CAN
// =========================================================
bool sendCAN(uint8_t axis_id, uint8_t cmd_id, const uint8_t* data, uint8_t len) {
    if (!sys.can_initialized) return false;
    twai_message_t msg;
    msg.identifier = (axis_id << 5) | cmd_id;
    msg.extd = 0; msg.rtr = 0; msg.data_length_code = len;
    for (int i = 0; i < len; i++) msg.data[i] = data[i];
    return (twai_transmit(&msg, pdMS_TO_TICKS(2)) == ESP_OK);
}

void setAxisState(uint8_t axis_id, uint8_t state) {
    uint32_t s = state;
    sendCAN(axis_id, CMD_SET_AXIS_STATE, (uint8_t*)&s, 4);
}

bool setTorque(uint8_t axis_id, float torque) {
    return sendCAN(axis_id, CMD_SET_INPUT_TRQ, (uint8_t*)&torque, 4);
}

void clearErrors(uint8_t axis_id) {
    sendCAN(axis_id, CMD_CLEAR_ERRORS, nullptr, 0);
}

void clearAllErrors() {
    Serial.println("CLEARING ALL ERRORS...");
    for (int i = 0; i < NUM_AXES; i++) { clearErrors(i); delay(5); }
}

void armSystem() {
    if (sys.state == STATE_ARMED || sys.state == STATE_FATAL_ERROR || !sys.can_initialized) return;
    Serial.println("ARMING...");
    for (int i = 0; i < NUM_AXES; i++) { setAxisState(i, AXIS_STATE_CLOSED_LOOP_CONTROL); delay(10); } 
    sys.state = STATE_ARMED;
    sys.neutral_locked = true;
    sys.current_torque = 0.0f;
    sys.target_torque  = 0.0f;
    outCfg.front_enabled = true;
    outCfg.rear_enabled = true;
}

void disarmSystem() {
    if (sys.state == STATE_DISARMED || sys.state == STATE_FATAL_ERROR) return;
    Serial.println("DISARMING...");
    for (int i = 0; i < NUM_AXES; i++) setTorque(i, 0.0f); 
    delay(20); 
    for (int i = 0; i < NUM_AXES; i++) { setAxisState(i, AXIS_STATE_IDLE); delay(10); }
    sys.state = STATE_DISARMED;
    sys.neutral_locked = true;
    sys.current_torque = 0.0f;
    sys.target_torque  = 0.0f;
    outCfg.front_enabled = false;
    outCfg.rear_enabled  = false;
    brakeLight.braking   = false;
}

void readCANMessages() {
    if (!sys.can_initialized) return;
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        uint8_t axis_id = (msg.identifier >> 5) & 0x3F;
        uint8_t cmd_id  = msg.identifier & 0x1F;
        if (axis_id >= NUM_AXES) continue;
        AxisTelemetry& ax = axis[axis_id];

        switch (cmd_id) {
            case CMD_HEARTBEAT:
                if (msg.data_length_code >= 8) {
                    memcpy(&ax.axis_error, msg.data, 4);
                    ax.axis_state = msg.data[4];
                }
                ax.connected = true;
                ax.last_heartbeat = millis();
                break;
            case CMD_GET_ENCODER_EST:
                if (msg.data_length_code >= 8) memcpy(&ax.vel_estimate, msg.data + 4, 4);
                break;
            case CMD_GET_IQ:
                if (msg.data_length_code >= 8) memcpy(&ax.iq_measured, msg.data + 4, 4);
                break;
            case CMD_GET_BUS_VI:
                if (msg.data_length_code >= 8) memcpy(&ax.bus_voltage, msg.data, 4);
                break;
            case CMD_GET_TEMP:
                if (msg.data_length_code >= 4) memcpy(&ax.temperature, msg.data, 4);
                break;
        }
    }
    for (int i = 0; i < NUM_AXES; i++) {
        if (axis[i].connected && millis() - axis[i].last_heartbeat > 2000) {
            axis[i].connected = false;
        }
    }
}

// =========================================================
// VX1 Parser & Protection
// =========================================================
uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc = crc << 1;
        }
    }
    return crc;
}

void parseVX1Packet() {
    static uint8_t buffer[256];
    static uint8_t buf_idx = 0;
    static bool in_packet  = false;

    while (VX1Serial.available()) {
        uint8_t b = VX1Serial.read();
        if (!in_packet) {
            if (b == 0x02 || b == 0x03) { buffer[0] = b; buf_idx = 1; in_packet = true; }
            continue;
        }
        if (buf_idx < sizeof(buffer)) { buffer[buf_idx++] = b; }
        else { in_packet = false; buf_idx = 0; continue; }

        if (b == 0x03 && buf_idx > 3) {
            in_packet = false;
            if (buf_idx < 6) { buf_idx = 0; continue; }
            uint8_t plen = buffer[1];
            uint16_t rcrc = (buffer[buf_idx-3] << 8) | buffer[buf_idx-2];
            
            if (rcrc == crc16(&buffer[2], plen)) {
                vx1.last_valid = millis();
                vx1.signal_ok  = true;

                if (buffer[2] == 0x23 && plen >= 3) {
                    int raw = (int)buffer[4];  
                    vx1.raw_adc = raw;
                    
                    int cc = vx1.cal_center, cmin = vx1.cal_min, cmax = vx1.cal_max;
                    float n;
                    if (!vx1.calibrated) {
                        n = ((float)raw - 127.0f) / 127.0f;
                    } else {
                        if (raw >= cc) n = (cmax > cc) ? (float)(raw - cc) / (float)(cmax - cc) : 0.0f;
                        else n = (cc > cmin) ? (float)(raw - cc) / (float)(cc - cmin) : 0.0f;
                    }
                    n = constrain(n, -1.0f, 1.0f);

                    if (fabsf(n) < DEADZONE) vx1.throttle = 0.0f;
                    else if (n > 0) vx1.throttle = (n - DEADZONE) / (1.0f - DEADZONE);
                    else vx1.throttle = (n + DEADZONE) / (1.0f - DEADZONE);
                    vx1.throttle = constrain(vx1.throttle, -1.0f, 1.0f);

                    // --- STUCK THROTTLE PROTECTION ---
                    if (!vx1.initial_check_done) {
                        vx1.initial_check_done = true;
                        if (fabsf(vx1.throttle) > 0.1f) {
                            Serial.println("FATAL ERROR: Throttle Stuck!");
                            sys.state = STATE_FATAL_ERROR;
                            disarmSystem();
                        }
                    }
                }
            }
            buf_idx = 0;
        }
    }

    if (millis() - vx1.last_valid > VX1_TIMEOUT_MS) {
        if (vx1.signal_ok && sys.state == STATE_ARMED) {
            Serial.println("VX1 LOST! Ramping down...");
            sys.state = STATE_RAMPING_DOWN;
            sys.target_torque = 0.0f;
        }
        vx1.signal_ok  = false;
        vx1.throttle   = 0.0f;
    } else if (vx1.signal_ok && sys.state == STATE_RAMPING_DOWN) {
        Serial.println("VX1 RECOVERED!");
        sys.state = STATE_ARMED;
    }
}

// =========================================================
// Gestures: Arming & Disarm/Clear Error holds
// =========================================================
void processGestures() {
    if (!vx1.signal_ok || sys.state == STATE_FATAL_ERROR) return;

    // --- 1. ARM GESTURE (Up -> Down -> Center) ---
    if (sys.state == STATE_DISARMED) {
        if (gesture.arm_phase == ARM_IDLE && vx1.throttle > 0.9f) {
            gesture.arm_phase = ARM_UP; gesture.arm_timer = millis();
        } 
        else if (gesture.arm_phase == ARM_UP && vx1.throttle < -0.9f) {
            if (millis() - gesture.arm_timer < 3000) gesture.arm_phase = ARM_DOWN;
            else gesture.arm_phase = ARM_IDLE;
        } 
        else if (gesture.arm_phase == ARM_DOWN && fabsf(vx1.throttle) < 0.1f) {
            if (millis() - gesture.arm_timer < 5000) armSystem();
            gesture.arm_phase = ARM_IDLE;
        }
    }

    // --- 2. HOLD TO DISARM & CLEAR ERRORS ---
    float max_vel = 0;
    for (int i = 0; i < NUM_AXES; i++) {
        if (axis[i].connected && fabsf(axis[i].vel_estimate) > max_vel) max_vel = fabsf(axis[i].vel_estimate);
    }
    
    // Курок полностью нажат назад И скейт стоит
    if (vx1.throttle < -0.8f && max_vel < BRAKE_VEL_THRESH) {
        if (!gesture.reverse_held) {
            gesture.reverse_held = true; gesture.reverse_timer = millis();
            gesture.disarm_triggered = false; gesture.clear_triggered = false;
        } else {
            unsigned long held_time = millis() - gesture.reverse_timer;
            // 2 секунды = Disarm
            if (held_time > 2000 && !gesture.disarm_triggered && sys.state == STATE_ARMED) {
                disarmSystem(); gesture.disarm_triggered = true;
            }
            // 5 секунд = Clear Errors
            if (held_time > 5000 && !gesture.clear_triggered) {
                clearAllErrors(); gesture.clear_triggered = true;
            }
        }
    } else {
        gesture.reverse_held = false;
    }
}

// =========================================================
// Main Update Logic
// =========================================================
void updateSystem() {
    static unsigned long last_t = 0;
    unsigned long now = millis();
    if (now - last_t < 20) return; // 50 Hz цикл
    float dt = (now - last_t) / 1000.0f;
    last_t = now;

    if (sys.state == STATE_FATAL_ERROR) {
        updateOutputs();
        return; 
    }

    // --- Снятие нейтрали (Neutral Lock) ---
    if (sys.neutral_locked && sys.state == STATE_ARMED && fabsf(vx1.throttle) < 0.05f) {
        sys.neutral_locked = false;
    }

    // --- LIMP MODE ---
    float max_voltage = 0;
    for(int i=0; i<NUM_AXES; i++) if(axis[i].connected && axis[i].bus_voltage > max_voltage) max_voltage = axis[i].bus_voltage;
    
    float current_max_fwd = MAX_TORQUE_FWD;
    if (max_voltage > 10.0f && max_voltage < BATT_V_LIMP) {
        float multiplier = map_float(max_voltage, BATT_V_MIN, BATT_V_LIMP, 0.3f, 1.0f);
        current_max_fwd = MAX_TORQUE_FWD * constrain(multiplier, 0.3f, 1.0f);
    }

    bool braking_detected = false;

    if (sys.state == STATE_ARMED && !sys.neutral_locked && vx1.signal_ok) {
        float t = vx1.throttle;

        if (t > 0.0f) {
            sys.target_torque = t * current_max_fwd;
        } 
        else if (t < 0.0f) {
            // --- NO REVERSE BRAKING ---
            float max_vel = 0;
            for (int i=0; i<NUM_AXES; i++) if (axis[i].connected && axis[i].vel_estimate > max_vel) max_vel = axis[i].vel_estimate;
            
            if (max_vel > BRAKE_VEL_THRESH) {
                sys.target_torque = t * MAX_TORQUE_BRAKE; // Тормозим (t < 0)
                braking_detected = true;
            } else {
                sys.target_torque = 0.0f; // Стоим
                braking_detected = true;  // Стоп-сигнал всё равно горит
            }
        } 
        else {
            sys.target_torque = 0.0f;
        }
    } else if (sys.state == STATE_RAMPING_DOWN) {
        sys.target_torque = 0.0f;
        braking_detected = true;
    } else {
        sys.target_torque = 0.0f;
    }

    // --- RAMPING (Ускорение/Замедление/Отвал связи) ---
    float diff = sys.target_torque - sys.current_torque;
    if (sys.state == STATE_RAMPING_DOWN) {
        float maxd = EMERGENCY_RAMP_RATE * dt;
        if (fabsf(diff) <= maxd) {
            sys.current_torque = sys.target_torque;
            disarmSystem(); 
        } else {
            sys.current_torque += (diff > 0) ? maxd : -maxd;
        }
    } else {
        float maxd = TORQUE_RAMP_RATE * dt;
        sys.current_torque += constrain(diff, -maxd, maxd);
    }

    // --- Стоп Сигнал ---
    if (braking_detected != brakeLight.braking) {
        brakeLight.braking = braking_detected;
        brakeLight.light_on = braking_detected;
        brakeLight.phase_start = now;
    }

    // --- CAN TX ---
    if (now - sys.last_can_tx >= 20 && sys.can_initialized) { // Отправка момента 50Гц
        sys.last_can_tx = now;
        for (int i = 0; i < NUM_AXES; i++) {
            if (!axis[i].connected) continue;
            
            // Smart Recovery - если в Armed ось выпала в ошибку/IDLE
            if ((sys.state == STATE_ARMED || sys.state == STATE_RAMPING_DOWN) && axis[i].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
                if (axis[i].axis_error != 0) clearErrors(i);
                else setAxisState(i, AXIS_STATE_CLOSED_LOOP_CONTROL);
                continue;
            }
            
            setTorque(i, sys.current_torque);
        }
    }

    updateOutputs();
}

void updateOutputs() {
    unsigned long now = millis();

    // 1. Авария - стробоскоп (оставляем как было)
    if (sys.state == STATE_FATAL_ERROR) {
        bool flash = (now % 200) < 100;
        ledcWrite(LEDC_CH_FRONT, flash ? 255 : 0);
        ledcWrite(LEDC_CH_REAR,  flash ? 255 : 0);
        return;
    }

    // --- ЛОГИКА МАЯЧКА ДЛЯ DISARM ---
    // Цикл 12 секунд: 10 сек выключено, 2 сек включено
    bool beacon_on = false;
    if (sys.state == STATE_DISARMED) {
        if ((now % 12000) >= 10000) { 
            beacon_on = true;
        }
    }

    // 2. Фара (Передний свет)
    if (sys.state == STATE_ARMED || sys.state == STATE_RAMPING_DOWN) {
        // Логика Limp Mode (пульсация при разряде)
        float max_voltage = 0;
        for(int i=0; i<NUM_AXES; i++) if(axis[i].connected && axis[i].bus_voltage > max_voltage) max_voltage = axis[i].bus_voltage;

        if (max_voltage > 10.0f && max_voltage < BATT_V_LIMP) {
            int breath = 127 + 127 * sin(now / 300.0f);
            ledcWrite(LEDC_CH_FRONT, breath);
        } else {
            ledcWrite(LEDC_CH_FRONT, outCfg.front_brightness);
        }
    } 
    else if (sys.state == STATE_DISARMED) {
        // Плавно включаем на 2 секунды раз в 10 секунд
        if (beacon_on) {
            // Рассчитываем плавность (0 -> 255 -> 0 за 2 секунды)
            float progress = (now % 12000 - 10000) / 2000.0f; 
            int brightness = 255 * sin(progress * PI); 
            ledcWrite(LEDC_CH_FRONT, brightness);
        } else {
            ledcWrite(LEDC_CH_FRONT, 0);
        }
    }

    // 3. Стоп-сигнал
    if (sys.state == STATE_ARMED || sys.state == STATE_RAMPING_DOWN) {
        bool is_braking = (sys.target_torque < -0.1f);
        if (is_braking) {
            bool blink = (now % (BRAKE_ON_MS + BRAKE_OFF_MS)) < BRAKE_ON_MS;
            ledcWrite(LEDC_CH_REAR, blink ? BRAKE_LIGHT_BRIGHT : 0);
        } else {
            ledcWrite(LEDC_CH_REAR, 0); // Твой предыдущий запрос (без габарита)
        }
    } 
    else if (sys.state == STATE_DISARMED) {
        if (beacon_on) {
            float progress = (now % 12000 - 10000) / 2000.0f;
            int brightness = 255 * sin(progress * PI);
            ledcWrite(LEDC_CH_REAR, brightness);
        } else {
            ledcWrite(LEDC_CH_REAR, 0);
        }
    }
}