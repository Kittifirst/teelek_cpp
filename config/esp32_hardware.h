#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

    //define your robot' specs here
    #define MOTOR_MAX_RPM 500                                               // motor's max RPM          
    #define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
    #define MOTOR_OPERATING_VOLTAGE 12                                      // motor's operating voltage (used to calculate max RPM)
    #define MOTOR_POWER_MAX_VOLTAGE 12                                      // max voltage of the motor's power source (used to calculate max RPM)
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                                 // current voltage reading of the power connected to the motor (used for calibration)
    #define ENCODER1_PULSES_PER_REVOLUTION 990                               // encoder 1 pulse
    #define ENCODER2_PULSES_PER_REVOLUTION 990                               // encoder 2 pulse
    #define ENCODER3_PULSES_PER_REVOLUTION 990                               // encoder 3 pulse
    #define ENCODER4_PULSES_PER_REVOLUTION 990                               // encoder 4 1pulse
    #define ENCODER5_PULSES_PER_REVOLUTION 990                               // encoder 5 1pulse
    #define ENCODER6_PULSES_PER_REVOLUTION 990                               // encoder 6 1pulse
    #define ENCODER_TICKS 20                                                 // encoder ticks
    #define COUNTS_PER_REV1 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel1 encoder's no of ticks per rev
    #define COUNTS_PER_REV2 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel2 encoder's no of ticks per rev
    #define COUNTS_PER_REV3 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel3 encoder's no of ticks per rev
    #define COUNTS_PER_REV4 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel4 encoder's no of ticks per rev
    #define WHEEL_DIAMETER 0.040                                           // wheel's diameter in meters
    // #define LR_WHEELS_DISTANCE 0.335                                        // distance between left and right wheels
    #define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min PWM_Max * -1
    #define GEAR_Ratio 1.575                                                // Midpoint of the PWM signal

    // #define FIRST_TCA_CHANNEL 3 // TCA9548A channel for the first AS5600 sensor
    // #define AS5600_COUNT 3 // Number of AS5600 sensors

    // INVERT MOTOR DIRECTIONS
    #define MOTOR1_INV false
    #define MOTOR2_INV true
    #define MOTOR3_INV false
    #define MOTOR4_INV true

    //  Motor Brake
    #define MOTOR1_BRAKE true
    #define MOTOR2_BRAKE true
    #define MOTOR3_BRAKE true
    #define MOTOR4_BRAKE true


    #ifdef teelek_karake

        // Motor 1 Parameters
        #define MOTOR1_PWM  -1
        #define MOTOR1_IN_A 4
        #define MOTOR1_IN_B 5

        // Motor 2 Parameters
        #define MOTOR2_PWM  -1
        #define MOTOR2_IN_A 15
        #define MOTOR2_IN_B 16

        // Motor 3 Parameters
        #define MOTOR3_PWM  -1
        #define MOTOR3_IN_A 1
        #define MOTOR3_IN_B 2

        // Motor 4 Parameters
        #define MOTOR4_PWM  -1
        #define MOTOR4_IN_A 38
        #define MOTOR4_IN_B 37

        // // Motor 1 Parameters
        // #define MOTOR1_PWM  -1
        // #define MOTOR1_IN_A -1
        // #define MOTOR1_IN_B -1

        // // Motor 2 Parameters
        // #define MOTOR2_PWM  -1
        // #define MOTOR2_IN_A -1
        // #define MOTOR2_IN_B -4

        // Motor 3 Parameters
        // #define MOTOR3_PWM  -1
        // #define MOTOR3_IN_A -1
        // #define MOTOR3_IN_B -1

        // // Motor 4 Parameters
        // #define MOTOR4_PWM  -1
        // #define MOTOR4_IN_A -1
        // #define MOTOR4_IN_B -1      



        // Encoder 1 Parameter
        #define MOTOR1_ENCODER_INCRIMENT -1
        #define MOTOR1_ENCODER_PIN_A 22
        #define MOTOR1_ENCODER_PIN_B 21
        #define MOTOR1_ENCODER_RATIO 1

        // Encoder 2 Parameter
        #define MOTOR2_ENCODER_INCRIMENT -1
        #define MOTOR2_ENCODER_PIN_A 19
        #define MOTOR2_ENCODER_PIN_B 18
        #define MOTOR2_ENCODER_RATIO 1

        // Encoder 3 Parameter
        #define MOTOR3_ENCODER_INCRIMENT -1
        #define MOTOR3_ENCODER_PIN_A 16
        #define MOTOR3_ENCODER_PIN_B 4
        #define MOTOR3_ENCODER_RATIO 1

        // Encoder 4 Parameter
        #define MOTOR4_ENCODER_INCRIMENT -1
        #define MOTOR4_ENCODER_PIN_A 5
        #define MOTOR4_ENCODER_PIN_B 17
        #define MOTOR4_ENCODER_RATIO 1
    #endif



    /*----------------------------------------------------------------------------------------------------------------------*/


    #ifdef teelek_katsu

        // Motor 5 Parameters
        #define MOTOR5_PWM  -1
        #define MOTOR5_IN_A 25
        #define MOTOR5_IN_B 26

        //  #define MOTOR5_PWM  -1
        // #define MOTOR5_IN_A -1
        // #define MOTOR5_IN_B -1


        // Motor 6 Parameters
        #define MOTOR6_PWM  -1
        #define MOTOR6_IN_A 32
        #define MOTOR6_IN_B 33

        // Motor 6 Parameters
        // #define MOTOR6_PWM  -1
        // #define MOTOR6_IN_A -1
        // #define MOTOR6_IN_B -1


        // Encoder 5 Parameter
        #define MOTOR5_ENCODER_INCRIMENT -1
        #define MOTOR5_ENCODER_PIN_A 19
        #define MOTOR5_ENCODER_PIN_B 18
        #define MOTOR5_ENCODER_RATIO 1

        // Encoder 6 Parameter
        #define MOTOR6_ENCODER_INCRIMENT -1
        #define MOTOR6_ENCODER_PIN_A 16
        #define MOTOR6_ENCODER_PIN_B 4
        #define MOTOR6_ENCODER_RATIO 1

        // Servo Parameter
        #define SERVO_BASE_PIN 13
        #define SERVO_TOP_PIN 23
        // #define SERVO1_PIN -1
        // #define SERVO2_PIN -1

        // Hall Sensor Parameter
        #define Hall_Sensor3 34

        #define IMU_RST 17
        #define IMU_INT 5


    #endif

    // INVERT ENCODER DIRECTIONS
    #define MOTOR1_ENCODER_INV true
    #define MOTOR2_ENCODER_INV false
    #define MOTOR3_ENCODER_INV true
    #define MOTOR4_ENCODER_INV false
    #define MOTOR5_ENCODER_INV true
    #define MOTOR6_ENCODER_INV false


    // I2C communication
    #define SCL_PIN 22
    #define SDA_PIN 21

#endif