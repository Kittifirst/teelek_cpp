#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

    //define your robot' specs here
    #define MOTOR_MAX_RPM 500                                               // motor's max RPM          
    #define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
    #define MOTOR_OPERATING_VOLTAGE 12                                      // motor's operating voltage (used to calculate max RPM)
    #define MOTOR_POWER_MAX_VOLTAGE 12                                      // max voltage of the motor's power source (used to calculate max RPM)
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                                 // current voltage reading of the power connected to the motor (used for calibration)

    // Encoder (ใช้แค่ 2 ข้าง สำหรับรูดเข้า rack)
    #define ENCODER_TICKS 11
    #define ENCODER1_PULSES_PER_REVOLUTION 990
    #define ENCODER2_PULSES_PER_REVOLUTION 990
    #define COUNTS_PER_REV1 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS
    #define COUNTS_PER_REV2 ENCODER2_PULSES_PER_REVOLUTION * ENCODER_TICKS

    #define ENCODER_TICKS_PER_REV1 COUNTS_PER_REV1
    #define ENCODER_TICKS_PER_REV2 COUNTS_PER_REV2
    
    // #define LR_WHEELS_DISTANCE 0.335                                        // distance between left and right wheels
    #define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min PWM_Max * -1
    // #define GEAR_Ratio 1.575                                               // Midpoint of the PWM signal
    #define WHEEL_DIAMETER 0.13                                           // wheel's diameter in meters 

    // INVERT MOTOR DIRECTIONS  
    #define MOTOR1_INV false
    #define MOTOR2_INV true
    #define MOTOR3_INV false
    #define MOTOR4_INV true
    #define MOTORLOAD_INV false

    //  Motor Brake
    #define MOTOR1_BRAKE true
    #define MOTOR2_BRAKE true
    #define MOTOR3_BRAKE true
    #define MOTOR4_BRAKE true
    #define MOTORLOAD_BRAKE true

    // Ultrasonic
    #define ULTRA_TRIG 12
    #define ULTRA_ECHO 13
    
    #ifdef teelek_karake

        // Motor 1 Parameters
        #define MOTOR1_PWM  -1
        #define MOTOR1_IN_A 4
        #define MOTOR1_IN_B 5 

        // Motor 2 Parameters
        #define MOTOR2_PWM  -1
        #define MOTOR2_IN_A 35 
        #define MOTOR2_IN_B 36 

        // Motor 3 Parameters
        #define MOTOR3_PWM  -1
        #define MOTOR3_IN_A 1 
        #define MOTOR3_IN_B 2 

        // Motor 4 Parameters
        #define MOTOR4_PWM  -1
        #define MOTOR4_IN_A 15 
        #define MOTOR4_IN_B 16

         // Encoder Rack Left
        #define CLK_L 20
        #define DT_L  21
        
        // Encoder Rack Left
        #define CLK_R 47
        #define DT_R  48
        
        // ---------------- Encoder 4 wheels ----------------

        // Motor 1
        #define Encoder_LF_A 41
        #define Encoder_LF_B 42

        // Motor 3
        #define Encoder_LB_A 39
        #define Encoder_LB_B 40

        // Motor 2
        #define Encoder_RF_A 13
        #define Encoder_RF_B 14

        // Motor 4
        #define Encoder_RB_A 38
        #define Encoder_RB_B 37


        #define GEAR_RATIO 534.0168 
        #define COUNTS_PER_REV ENCODER_TICKS * GEAR_RATIO * 4       // encoder resolution
        #define ENCODER_INV_LF true
        #define ENCODER_INV_LB false
        #define ENCODER_INV_RF true
        #define ENCODER_INV_RB false
    #endif

    #ifdef teelek_katsu

        // Motor load config
        #define MOTORLOAD_PWM  -1
        #define MOTORLOAD_IN_A  4
        #define MOTORLOAD_IN_B  5

        // Servo Parameter
        #define SERVO_PIN 13
        #define SERVO_MIN_ANGLE 0
        #define SERVO_MAX_ANGLE 180
    #endif

    // INVERT ENCODER DIRECTIONS (ใช้เฉพาะ 2 ข้าง)
    #define MOTOR1_ENCODER_INV false
    #define MOTOR2_ENCODER_INV false

    // I2C communication
    #define SCL_PIN 8
    #define SDA_PIN 21

#endif
