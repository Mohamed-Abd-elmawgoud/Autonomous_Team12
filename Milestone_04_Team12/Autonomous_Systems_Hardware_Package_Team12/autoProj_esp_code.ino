/*
 * esp_bridge_receiver_rtos.ino  –  ESP32-S3 version (RTOS + Binary TX/RX)
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// ================= FREE RTOS =================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define STEER_PIN       18
#define MOTOR_ENA_PIN   9
#define MOTOR_IN1_PIN   10
#define MOTOR_IN2_PIN   11

// ── Encoder pins ──────────────────────────────────────────────────────────────
#define ENCODER_PIN_A   12 // Interrupt pin
#define ENCODER_PIN_B   13 // Direction pin

// ── I2C / IMU pins ────────────────────────────────────────────────────────────
#define I2C_SDA_PIN     15 
#define I2C_SCL_PIN     16

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

// ── RX Packet constants (From Pi to ESP) ──────────────────────────────────────
#define RX_PKT_LEN      8
#define RX_START1       0xAA
#define RX_START2       0x55
#define RX_END_BYTE     0x0A

// ── TX Packet constants (From ESP to Pi) ──────────────────────────────────────
#define TX_PKT_LEN      12
#define TX_START1       0xBB
#define TX_START2       0x66
#define TX_END_BYTE     0x0A

// ── Timings & Safety ──────────────────────────────────────────────────────────
#define PACKET_TIMEOUT_MS   500
#define TASK_PERIOD_MS      10  // 20ms period for both Sensors and Comms

// ── Globals & Objects ─────────────────────────────────────────────────────────
Servo steerServo;
MPU6050 mpu;

// UART Parsing Buffer
uint8_t buf[RX_PKT_LEN];
uint8_t idx = 0;
unsigned long lastPacketTime = 0;

// Motor Command State
uint16_t cmd_thr_us = THR_NEUTRAL_US;
uint8_t cmd_steer_deg = STEER_CTR_DEG;

// ── Encoder Variables ─────────────────────────────────────────────────────────
volatile long encoderTicks = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ── IMU Variables ─────────────────────────────────────────────────────────────
float gz_bias = 0;

// ── RTOS Shared Variables & Synchronization ───────────────────────────────────
SemaphoreHandle_t dataMutex;

TaskHandle_t sensorTaskHandle;
TaskHandle_t commTaskHandle;

// Shared Sensor Data
long shared_ticks = 0;
float shared_z_angle = 0.0;


// ==============================================================================
// ========================= ENCODER INTERRUPT ==================================
// ==============================================================================
void IRAM_ATTR readEncoderISR() {
    portENTER_CRITICAL_ISR(&timerMux);
    if (digitalRead(ENCODER_PIN_B) == HIGH) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ==============================================================================
// ========================= GYRO CALIBRATION ===================================
// ==============================================================================
void calibrateGyro() {
    Serial.println("INFO: Calibrating Gyro (Keep still)...");
    delay(1000); 
    long sum = 0;
    const int num_samples = 500;

    for (int i = 0; i < num_samples; i++) {
        // ONLY read the Z-axis gyroscope
        int16_t gz = mpu.getRotationZ();
        sum += gz;
        delay(3);
    }

    gz_bias = (float)sum / num_samples;
    Serial.printf("INFO: Gyro calibration done. Z Bias: %.2f\n", gz_bias);
}
// ==============================================================================
// ========================= SETUP ==============================================
// ==============================================================================
void setup() {
    Serial.begin(115200);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // 1. Encoder Setup
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoderISR, RISING);

    //2. Servo Setup
    
    steerServo.attach(STEER_PIN, 500, 2400);
    steerServo.write(STEER_CTR_DEG);

    // 3. Motor Setup
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);

    ledcAttach(MOTOR_ENA_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcWrite(MOTOR_ENA_PIN, 0);
    delay(500);
    stopMotor();

    // 3. I2C & IMU Setup
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERR: MPU6050 connection failed!");
        while(1); 
    } else {
        Serial.println("INFO: MPU6050 connected.");
        calibrateGyro();
    }

    // 4. RTOS Setup
    dataMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 3, &sensorTaskHandle, 0);
    xTaskCreatePinnedToCore(TaskCommsControl, "CommsControl", 4096, NULL, 2, &commTaskHandle, 1);
}

// ==============================================================================
// ========================= MAIN LOOP ==========================================
// ==============================================================================
void loop() {
    vTaskDelay(portMAX_DELAY);
}

// ==============================================================================
// ========================= RTOS TASKS =========================================
// ==============================================================================
// ── CORE 0: Sensor Acquisition & Integration (20ms interval) ──────────────────
void TaskSensors(void *pv) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_PERIOD_MS);
    
    unsigned long last_micros = micros();
    float current_z_angle = 0.0;

    while (1) {
        // 1. Calculate dt
        unsigned long now = micros();
        float dt = (now - last_micros) / 1000000.0; 
        last_micros = now;

        // 2. Read ONLY the Z-axis from MPU6050
        int16_t gz = mpu.getRotationZ();

        // 3. Compute Heading
        float gz_rate = (gz - gz_bias) / 131.0; 
        if (abs(gz_rate) < 0.5) { gz_rate = 0; } // Deadband filter

        current_z_angle += (gz_rate * dt);

        if (current_z_angle > 180.0)  current_z_angle -= 360.0;
        if (current_z_angle < -180.0) current_z_angle += 360.0;
   
        Serial.println(current_z_angle);
     
        
        // 4. Safely grab volatile encoder ticks
        long t_ticks;
        portENTER_CRITICAL(&timerMux);
        t_ticks = encoderTicks;
        portEXIT_CRITICAL(&timerMux);

        // 5. Lock Mutex and update shared variables
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        shared_ticks = t_ticks;
        shared_z_angle = current_z_angle;
        xSemaphoreGive(dataMutex);

        // 6. Sleep precisely
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// ── CORE 1: Comm, Processing, & Control (20ms interval) ───────────────────────
void TaskCommsControl(void *pv) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_PERIOD_MS);

    while (1) {
        // 1. Read Serial Data (Non-blocking)
        while (Serial2.available()) {
            uint8_t b = Serial2.read();
            if (idx == 0) {
                if (b != RX_START1) continue;
            } else if (idx == 1) {
                if (b != RX_START2) { idx = 0; continue; }
            }

            buf[idx++] = b;

            if (idx == RX_PKT_LEN) {
                idx = 0;
                if (buf[7] == RX_END_BYTE) {
                    uint8_t csum = buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
                    if (csum == buf[6]) {
                        cmd_steer_deg = constrain(buf[2], STEER_MIN_DEG, STEER_MAX_DEG);
                        cmd_thr_us    = constrain((uint16_t)buf[4] | ((uint16_t)buf[5] << 8), THR_MIN_US, THR_MAX_US);
                        lastPacketTime = millis();
                    }
                }
            }
        }

        // 2. Safety Timeout
        if (lastPacketTime != 0 && (millis() - lastPacketTime > PACKET_TIMEOUT_MS)) {
            cmd_thr_us = THR_NEUTRAL_US;
            cmd_steer_deg = STEER_CTR_DEG;
            lastPacketTime = millis(); 
        }

        // 3. Actuate Motors
        steerServo.write(cmd_steer_deg);
        setDCMotor(cmd_thr_us);

        // 4. Get Sensor Data from Core 0
        long t_ticks;
        float t_z_angle;
        
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        t_ticks = shared_ticks;
        t_z_angle = shared_z_angle;
        xSemaphoreGive(dataMutex);

        // 5. Build and Send Binary Telemetry Packet
        uint8_t tx_buf[TX_PKT_LEN];
        
        // Start bytes
        tx_buf[0] = TX_START1; 
        tx_buf[1] = TX_START2;

        // Payload: Copy the 4 bytes of the 'long' and 4 bytes of the 'float' into the buffer
        memcpy(&tx_buf[2], &t_ticks, 4);
        memcpy(&tx_buf[6], &t_z_angle, 4);

        // Checksum: XOR of payload bytes (bytes 2 to 9)
        uint8_t tx_csum = 0;
        for(int i = 2; i < 10; i++) {
            tx_csum ^= tx_buf[i];
        }
        tx_buf[10] = tx_csum;

        // End byte
        tx_buf[11] = TX_END_BYTE;

        // Transmit over UART2
        Serial2.write(tx_buf, TX_PKT_LEN);

        // 6. Sleep precisely until next 20ms period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// ==============================================================================
// ========================= MOTOR HELPER FUNCTIONS =============================
// ==============================================================================

void stopMotor() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    ledcWrite(MOTOR_ENA_PIN, 0);
}

void setDCMotor(uint16_t thr_us) {
    if (thr_us <= THR_NEUTRAL_US) {
        uint16_t span  = THR_NEUTRAL_US - THR_MIN_US;
        uint16_t delta = THR_NEUTRAL_US - constrain(thr_us, THR_MIN_US, THR_NEUTRAL_US);
        uint8_t  duty  = (uint8_t)map(delta, 0, span, 0, 255);

        if (duty < 30) {
            stopMotor();
        } else {
            digitalWrite(MOTOR_IN1_PIN, LOW);
            digitalWrite(MOTOR_IN2_PIN, HIGH);
            ledcWrite(MOTOR_ENA_PIN, duty);
        }
    } else {
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
