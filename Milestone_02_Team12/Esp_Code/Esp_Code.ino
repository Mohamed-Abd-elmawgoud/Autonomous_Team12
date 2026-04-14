/*
 * esp_bridge_receiver.ino  –  ESP32-S3 version
 */

#include <ESP32Servo.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
#define STEER_PIN       16
#define MOTOR_ENA_PIN   9
#define MOTOR_IN1_PIN   10
#define MOTOR_IN2_PIN   11

// ── UART2 pins ────────────────────────────────────────────────────────────────
#define UART_BAUD       115200
#define UART_RX_PIN     4
#define UART_TX_PIN     5

// ── Servo limits ──────────────────────────────────────────────────────────────
#define STEER_MIN_DEG   60
#define STEER_MAX_DEG   120
#define STEER_CTR_DEG   90

// ── Throttle range (µs) ───────────────────────────────────────────────────────
#define THR_MIN_US      1000
#define THR_MAX_US      2000
#define THR_NEUTRAL_US  1500

// ── PWM settings ──────────────────────────────────────────────────────────────
#define MOTOR_PWM_FREQ  1000
#define MOTOR_PWM_RES   8

// ── Packet constants ──────────────────────────────────────────────────────────
#define PKT_LEN         8
#define START1          0xAA
#define START2          0x55
#define END_BYTE        0x0A

// ── Safety timeout ────────────────────────────────────────────────────────────
#define PACKET_TIMEOUT_MS 500

// ── Globals ───────────────────────────────────────────────────────────────────
Servo steerServo;
uint8_t buf[PKT_LEN];
uint8_t idx = 0;
unsigned long lastPacketTime = 0;

// ── Stop motor helper ─────────────────────────────────────────────────────────
void stopMotor() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    ledcWrite(MOTOR_ENA_PIN, 0);
}

// ── DC Motor ───────────────────────────────────────────────────────────────────
void setDCMotor(uint16_t thr_us) {
    if (thr_us <= THR_NEUTRAL_US) {
        uint16_t span  = THR_NEUTRAL_US - THR_MIN_US;
        uint16_t delta = THR_NEUTRAL_US - constrain(thr_us, THR_MIN_US, THR_NEUTRAL_US);
        uint8_t  duty  = (uint8_t)map(delta, 0, span, 0, 255);

        if (duty < 30) {
            stopMotor();
        } else {
            // Reverse
            digitalWrite(MOTOR_IN1_PIN, LOW);
            digitalWrite(MOTOR_IN2_PIN, HIGH);
            ledcWrite(MOTOR_ENA_PIN, duty);
        }
    } else {
        // Forward
        uint16_t span  = THR_MAX_US - THR_NEUTRAL_US;
        uint16_t delta = constrain(thr_us, THR_NEUTRAL_US, THR_MAX_US) - THR_NEUTRAL_US;
        uint8_t  duty  = (uint8_t)map(delta, 0, span, 0, 255);

        if (duty < 30) {
            stopMotor();
        } else {
            digitalWrite(MOTOR_IN1_PIN, HIGH);
            digitalWrite(MOTOR_IN2_PIN, LOW);
            ledcWrite(MOTOR_ENA_PIN, duty);
        }
    }
}

// ── Process packet ─────────────────────────────────────────────────────────────
void processPacket(uint8_t *pkt) {
    lastPacketTime = millis();

    uint8_t  steer_deg = pkt[2];
    uint16_t thr_us    = (uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8);

    steer_deg = constrain(steer_deg, STEER_MIN_DEG, STEER_MAX_DEG);
    thr_us    = constrain(thr_us,    THR_MIN_US,    THR_MAX_US);

    steerServo.write(steer_deg);
    setDCMotor(thr_us);

    Serial.printf("PKT OK  steer=%d°  thr_us=%d\n", steer_deg, thr_us);
}

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-S3 bridge starting...");

    // UART2
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // DC motor pins — configure and stop BEFORE servo attach
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);

    ledcAttach(MOTOR_ENA_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcWrite(MOTOR_ENA_PIN, 0);

    // Servo — attach after motor is already stopped
    steerServo.attach(STEER_PIN);
    steerServo.write(STEER_CTR_DEG);

    // Hold stop state for 500ms to let everything settle
    delay(500);
    stopMotor();

    Serial.println("Ready – waiting for packets");
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
    // Safety timeout — stop if no packet received within 500ms
    if (lastPacketTime != 0 && (millis() - lastPacketTime > PACKET_TIMEOUT_MS)) {
        stopMotor();
        steerServo.write(STEER_CTR_DEG);
        lastPacketTime = millis();   // prevent repeated log spam
        Serial.println("TIMEOUT: no packet — motors stopped");
    }

    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        if (idx == 0) {
            if (b != START1) return;
        } else if (idx == 1) {
            if (b != START2) { idx = 0; return; }
        }

        buf[idx++] = b;

        if (idx == PKT_LEN) {
            idx = 0;

            if (buf[7] != END_BYTE) {
                Serial.println("ERR: bad end byte");
                return;
            }

            uint8_t csum = buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
            if (csum != buf[6]) {
                Serial.printf("ERR: checksum 0x%02X expected 0x%02X\n", csum, buf[6]);
                return;
            }

            processPacket(buf);
        }
    }
}