#include <Arduino.h>
#include "motorDrive.h"

void setup() {
    Serial.begin(115200);
    motorDrive_begin();
    Serial.println("ESP32 4WD Ready");
}

void loop() {
    motorDrive_handleSerialOnce();
    motorDrive_update();

    // debug serial PWM
    Serial.printf("PWM L:%0.2f %0.2f | R:%0.2f %0.2f\n",
                  out_LF, out_LR, out_RF, out_RR);
    delay(20);
}
