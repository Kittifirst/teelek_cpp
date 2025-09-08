#include "motorDrive.h"

// ===== Shared variables =====
String rx_line;
bool estop = false;
uint32_t last_cmd_ms = 0;

float tgt_LF=0, tgt_LR=0, tgt_RF=0, tgt_RR=0;
float out_LF=0, out_LR=0, out_RF=0, out_RR=0;

static bool trackToggle = false;

// ===== PWM Utils =====
static void writeMotorPWM(int ch_in1, int ch_in2, float val) {
    int duty_fwd=0, duty_rev=0;
    int duty = int(abs(val) * PWM_MAX_DUTY);
    if (val > 0) duty_fwd = duty;
    else if (val < 0) duty_rev = duty;
    ledcWrite(ch_in1, duty_fwd);
    ledcWrite(ch_in2, duty_rev);
}

// ===== init =====
void motorDrive_begin() {
    ledcSetup(CH_LF_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_LF_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_LR_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_LR_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_RF_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_RF_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_RR_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_RR_IN2, PWM_FREQ_HZ, PWM_RES_BITS);

    ledcAttachPin(LF_IN1, CH_LF_IN1);
    ledcAttachPin(LF_IN2, CH_LF_IN2);
    ledcAttachPin(LR_IN1, CH_LR_IN1);
    ledcAttachPin(LR_IN2, CH_LR_IN2);
    ledcAttachPin(RF_IN1, CH_RF_IN1);
    ledcAttachPin(RF_IN2, CH_RF_IN2);
    ledcAttachPin(RR_IN1, CH_RR_IN1);
    ledcAttachPin(RR_IN2, CH_RR_IN2);

    last_cmd_ms = millis();
}

// ===== VW -> target wheels =====
void cmdVW_to_targets(float V_mps, float W_radps) {
    float vL = V_mps - (W_radps * WHEEL_SEP / 2.0f);
    float vR = V_mps + (W_radps * WHEEL_SEP / 2.0f);

    tgt_LF = constrain(vL / (WHEEL_RADIUS * MAX_OMEGA_FOR_FULL), -1,1);
    tgt_LR = tgt_LF;
    tgt_RF = constrain(vR / (WHEEL_RADIUS * MAX_OMEGA_FOR_FULL), -1,1);
    tgt_RR = tgt_RF;

    last_cmd_ms = millis();
}

// ===== handle serial =====
void motorDrive_handleSerialOnce() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c=='\n' || c=='\r') {
            if (rx_line.length()) {
                if (rx_line.startsWith("VW")) {
                    float v,w;
                    sscanf(rx_line.c_str(),"VW %f %f",&v,&w);
                    cmdVW_to_targets(v,w);
                } else if (rx_line.startsWith("P")) {
                    float lf,lr,rf,rr;
                    sscanf(rx_line.c_str(),"P %f %f %f %f",&lf,&lr,&rf,&rr);
                    tgt_LF=lf;tgt_LR=lr;tgt_RF=rf;tgt_RR=rr;
                    last_cmd_ms = millis();
                } else if (rx_line.startsWith("ESTOP")) {
                    estop = true;
                } else if (rx_line.startsWith("BTN TRACK")) {
                    trackToggle = true;
                }
                rx_line="";
            }
        } else rx_line += c;
    }
}

// ===== update PWM =====
void motorDrive_update() {
    uint32_t now = millis();
    if ((now-last_cmd_ms) > CMD_TIMEOUT_MS) {
        tgt_LF *= IDLE_DECAY;
        tgt_LR *= IDLE_DECAY;
        tgt_RF *= IDLE_DECAY;
        tgt_RR *= IDLE_DECAY;
    }

    auto ramp = [](float &out, float tgt){
        if (fabs(tgt-out) > RAMP_STEP) out += (tgt>out?RAMP_STEP:-RAMP_STEP);
        else out = tgt;
    };

    ramp(out_LF, tgt_LF);
    ramp(out_LR, tgt_LR);
    ramp(out_RF, tgt_RF);
    ramp(out_RR, tgt_RR);

    // write PWM
    writeMotorPWM(CH_LF_IN1, CH_LF_IN2, INVERT_LF?-out_LF:out_LF);
    writeMotorPWM(CH_LR_IN1, CH_LR_IN2, INVERT_LR?-out_LR:out_LR);
    writeMotorPWM(CH_RF_IN1, CH_RF_IN2, INVERT_RF?-out_RF:out_RF);
    writeMotorPWM(CH_RR_IN1, CH_RR_IN2, INVERT_RR?-out_RR:out_RR);
}

bool motorDrive_consumeTrackToggle() {
    if (trackToggle) { trackToggle=false; return true; }
    return false;
}
