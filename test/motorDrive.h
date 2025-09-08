#pragma once
#include <Arduino.h>

// ---------------- ขามอเตอร์ 4 ล้อ ----------------
#define LF_IN1 14
#define LF_IN2 12
#define LR_IN1 27
#define LR_IN2 26
#define RF_IN1 33
#define RF_IN2 25
#define RR_IN1 4
#define RR_IN2 0

// ---------------- PWM (LEDC) ----------------
static constexpr int PWM_FREQ_HZ  = 20000;     // 20 kHz
static constexpr int PWM_RES_BITS = 10;        // 10 bit
static constexpr int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// LEDC channel แยก IN1/IN2
static constexpr int CH_LF_IN1 = 0;
static constexpr int CH_LF_IN2 = 1;
static constexpr int CH_LR_IN1 = 2;
static constexpr int CH_LR_IN2 = 3;
static constexpr int CH_RF_IN1 = 4;
static constexpr int CH_RF_IN2 = 5;
static constexpr int CH_RR_IN1 = 6;
static constexpr int CH_RR_IN2 = 7;

// ---------------- Robot Parameters ----------------
static constexpr float WHEEL_SEP    = 0.20f;   // ระยะล้อซ้าย-ขวา (m)
static constexpr float WHEEL_RADIUS = 0.05f;   // รัศมีล้อ (m)
static constexpr float MAX_OMEGA_FOR_FULL = 20.0f; // rad/s -> 100% PWM

static constexpr bool INVERT_LF = false;
static constexpr bool INVERT_LR = false;
static constexpr bool INVERT_RF = false;
static constexpr bool INVERT_RR = false;

// Safety / slew
static constexpr uint32_t CMD_TIMEOUT_MS = 2000; // ms
static constexpr float    RAMP_STEP      = 0.05f;
static constexpr float    IDLE_DECAY     = 0.85f;

// ---------------- Shared Variables ----------------
extern String rx_line;
extern bool estop;
extern uint32_t last_cmd_ms;

extern float tgt_LF, tgt_LR, tgt_RF, tgt_RR; // target PWM (-1..1)
extern float out_LF, out_LR, out_RF, out_RR; // output PWM after ramp

// ---------------- Public API ----------------
void motorDrive_begin();            // init PWM
void motorDrive_handleSerialOnce(); // อ่านคำสั่ง Serial
void motorDrive_update();           // watchdog + ramp + write PWM
void cmdVW_to_targets(float V_mps, float W_radps);

// ---------- ปุ่ม one-shot ----------
bool motorDrive_consumeTrackToggle();
