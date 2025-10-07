#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>
#include <Encoder.h>

#include <esp32_Encoder.h>     //Encoder 4 wheels
#include <ESP32Servo.h>
#include <esp32_hardware.h>

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
    #define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
    #define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)
    
    //------------------------------ < Define > -------------------------------------//

rcl_publisher_t debug_cmd_vel_publisher;
rcl_subscription_t cmd_vel_subscriber;

geometry_msgs__msg__Twist debug_wheel_motor_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t debug_encoder_publisher;
geometry_msgs__msg__Twist debug_encoder_msg;

rcl_publisher_t debug_encoder_wheels_publisher;
std_msgs__msg__Float32MultiArray debug_encoder_wheels_msg;

rcl_subscription_t cmd_resetencoder_subscriber;
geometry_msgs__msg__Twist cmd_resetencoder_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
static unsigned long last_sync = 0;

// Encoder rack
float rack_tick = 0;
long last_L_tick = 0;
long last_R_tick = 0;

// Encoder wheels
float wheel_ticks[4] = {0};   // LF, LB, RF, RB
long lastLF = 0;
long lastLB = 0;
long lastRF = 0;
long lastRB = 0;

long offsetLF = 0, offsetLB = 0, offsetRF = 0, offsetRB = 0;
long totalLF = 0, totalLB = 0, totalRF = 0, totalRB = 0; // ✅ tick สะสม


enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


Controller motor1(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Controller motor3(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Controller motor4(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

// Encoder encoderL(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, false, 26.0f/20.0f);
// Encoder encoderR(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, false, 26.0f/20.0f);

// Encoder 4 wheels
esp32_Encoder encLF(Encoder_LF_A, Encoder_LF_B, COUNTS_PER_REV, ENCODER_INV_LF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encLB(Encoder_LB_A, Encoder_LB_B, COUNTS_PER_REV, ENCODER_INV_LB, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRF(Encoder_RF_A, Encoder_RF_B, COUNTS_PER_REV, ENCODER_INV_RF, GEAR_RATIO, WHEEL_DIAMETER);
esp32_Encoder encRB(Encoder_RB_A, Encoder_RB_B, COUNTS_PER_REV, ENCODER_INV_RB, GEAR_RATIO, WHEEL_DIAMETER);

//------------------------------ < Fuction Prototype > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

void cmd_vel_callback(const void *);
void cmd_load_callback(const void *);
void cmd_reset_encoder_callback(const void *msgin);

void publishData();
void getEncoderWheelsTick(); 
void getEncoderData();
void MovePower(int, int, int, int);
void load(int);
//------------------------------ < Main > -------------------------------------//


// ---------------- Encoder Left ----------------
#define CLK_L 20
#define DT_L  21
volatile long encoderL_pos = 0;
volatile uint8_t lastStateL = 0;

void IRAM_ATTR handleEncoderL() {
    uint8_t state = (digitalRead(CLK_L) << 1) | digitalRead(DT_L);
    int8_t change = 0;
    switch (lastStateL) {
        case 0b00: if (state == 0b01) change = 1; else if (state == 0b10) change = -1; break;
        case 0b01: if (state == 0b11) change = 1; else if (state == 0b00) change = -1; break;
        case 0b11: if (state == 0b10) change = 1; else if (state == 0b01) change = -1; break;
        case 0b10: if (state == 0b00) change = 1; else if (state == 0b11) change = -1; break;
    }
    encoderL_pos += change;
    lastStateL = state;
}

long readEncoderL() {
    noInterrupts();
    long v = encoderL_pos;
    interrupts();
    return v;
}

// ---------------- Encoder Right ----------------
#define CLK_R 47
#define DT_R  48
volatile long encoderR_pos = 0;
volatile uint8_t lastStateR = 0;

void IRAM_ATTR handleEncoderR() {
    uint8_t state = (digitalRead(CLK_R) << 1) | digitalRead(DT_R);
    int8_t change = 0;
    switch (lastStateR) {
        case 0b00: if (state == 0b01) change = 1; else if (state == 0b10) change = -1; break;
        case 0b01: if (state == 0b11) change = 1; else if (state == 0b00) change = -1; break;
        case 0b11: if (state == 0b10) change = 1; else if (state == 0b01) change = -1; break;
        case 0b10: if (state == 0b00) change = 1; else if (state == 0b11) change = -1; break;
    }
    encoderR_pos += change;
    lastStateR = state;
}

long readEncoderR() {
    noInterrupts();
    long v = encoderR_pos;
    interrupts();
    return v;
}



void setup()
{
    // Encoder Left
    pinMode(CLK_L, INPUT_PULLUP);
    pinMode(DT_L, INPUT_PULLUP);
    lastStateL = (digitalRead(CLK_L) << 1) | digitalRead(DT_L);
    attachInterrupt(digitalPinToInterrupt(CLK_L), handleEncoderL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DT_L), handleEncoderL, CHANGE);

    // Encoder Right
    pinMode(CLK_R, INPUT_PULLUP);
    pinMode(DT_R, INPUT_PULLUP);
    lastStateR = (digitalRead(CLK_R) << 1) | digitalRead(DT_R);
    attachInterrupt(digitalPinToInterrupt(CLK_R), handleEncoderR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DT_R), handleEncoderR, CHANGE);
    
    Serial.begin(115200);
    #ifdef MICROROS_WIFI
        
        IPAddress agent_ip(AGENT_IP);
        uint16_t agent_port = AGENT_PORT;
        set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
    #else
        set_microros_serial_transports(Serial);
    #endif
}

void loop()
{   
    if (millis() - last_sync > 10000) {
        syncTime();
        last_sync = millis();
    }
    switch (state)
    {
    case WAITING_AGENT:
        // EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 4)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(250, state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(150));
        }
        break;
    case AGENT_DISCONNECTED:
        MovePower(0, 0, 0, 0);
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//


// Motor Move

void MovePower(int Motor1Speed, int Motor2Speed, int Motor3Speed, int Motor4Speed)
{
    Motor1Speed = constrain(Motor1Speed, PWM_Min, PWM_Max);
    Motor2Speed = constrain(Motor2Speed, PWM_Min, PWM_Max);
    Motor3Speed = constrain(Motor3Speed, PWM_Min, PWM_Max);
    Motor4Speed = constrain(Motor4Speed, PWM_Min, PWM_Max);

    motor1.spin(Motor1Speed);
    motor2.spin(Motor2Speed);
    motor3.spin(Motor3Speed);
    motor4.spin(Motor4Speed);
}

void cmd_vel_callback(const void *msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float V_x = msg->linear.x;
    float W_z = msg->angular.z;
    float d = max(abs(V_x) + abs(W_z), (float) PWM_Max);
    int fl = (V_x - W_z)/d * (float) PWM_Max;
    int fr = (V_x + W_z)/d * (float) PWM_Max;
    int bl = (V_x - W_z)/d * (float) PWM_Max;
    int br = (V_x + W_z)/d * (float) PWM_Max;
    MovePower(fl, fr,
              bl, br);
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
    {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
        getEncoderData();
        getEncoderWheelsTick();
        publishData(); 
    }
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);
    
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    
    // create node
    #ifdef teelek_karake
        RCCHECK(rclc_node_init_default(&node, "teelek_karake", "", &support));
    #elif teelek_katsu
        RCCHECK(rclc_node_init_default(&node, "teelek_katsu", "", &support));
    #endif

    // Publishers
    #ifdef teelek_karake
    #elif teelek_katsu
    #endif

    RCCHECK(rclc_publisher_init_default(
        &debug_cmd_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/cmd_vel"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/teelek/cmd_move"));


    RCCHECK(rclc_publisher_init_default(
        &debug_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "teelek/debug/encoder"));

    RCCHECK(rclc_publisher_init_default(
        &debug_encoder_wheels_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "teelek/debug/encoder_wheels"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_resetencoder_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/teelek/cmd_resetencoder"));

    // ภายใน createEntities() หลังจากประกาศ publisher
    rosidl_runtime_c__float__Sequence__init(&debug_encoder_wheels_msg.data, 4);

        
    // create timer for control loop 1000/20 Hz
    const unsigned int control_timeout = 30;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        &controlCallback));
        
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));
 
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_resetencoder_subscriber,
        &cmd_resetencoder_msg,
        &cmd_reset_encoder_callback,
        ON_NEW_DATA));

    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_subscription_fini(&cmd_vel_subscriber, &node);

    rcl_publisher_fini(&debug_cmd_vel_publisher, &node);
    
    rcl_publisher_fini(&debug_encoder_publisher, &node);

    rcl_publisher_fini(&debug_encoder_wheels_publisher, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void getEncoderData()
{
    // อ่านค่าจาก encoder ซ้ายและขวา
    long curL = readEncoderL();
    long curR = -readEncoderR();

    // คำนวณ ticks ที่เปลี่ยนไป
    long diff_L_ticks = curL - last_L_tick;
    long diff_R_ticks = curR - last_R_tick;

    // อัปเดตค่าเก็บ tick ล่าสุด
    last_L_tick = curL;
    last_R_tick = curR;

    // ---------------- Logic เลือกฝั่งที่เชื่อถือได้ ----------------
    if (abs(diff_L_ticks) > 3 && abs(diff_L_ticks) >= abs(diff_R_ticks)) {
        // ถ้าฝั่งซ้ายมีการเปลี่ยนแปลงชัดเจน → ใช้ซ้ายเป็นตัวแทน
        diff_R_ticks = diff_L_ticks;
        curR = curL;
    } 
    else if (abs(diff_R_ticks) > 3 && abs(diff_R_ticks) > abs(diff_L_ticks)) {
        // ถ้าฝั่งขวามีการเปลี่ยนแปลงชัดเจน → ใช้ขวาเป็นตัวแทน
        diff_L_ticks = diff_R_ticks;
        curL = curR;
    }

    // ใช้ diff_L_ticks เป็น rack_tick เพราะ synced แล้ว
    rack_tick = diff_L_ticks;   

    // ส่งค่าออก ROS debug
    debug_encoder_msg.linear.x = curL;      
    debug_encoder_msg.linear.y = curR;      
    debug_encoder_msg.linear.z = rack_tick; 
}

void getEncoderWheelsTick() {
    long curLF = encLF.read();
    long curLB = encLB.read();
    long curRF = encRF.read();
    long curRB = encRB.read();

    debug_encoder_wheels_msg.data.data[0] = curLF - offsetLF;
    debug_encoder_wheels_msg.data.data[1] = curLB - offsetLB;
    debug_encoder_wheels_msg.data.data[2] = curRF - offsetRF;
    debug_encoder_wheels_msg.data.data[3] = curRB - offsetRB;
}


void resetEncoderOffset()
{
    offsetLF = encLF.read();
    offsetLB = encLB.read();
    offsetRF = encRF.read();
    offsetRB = encRB.read();

    // Set current message to zero
    debug_encoder_wheels_msg.data.data[0] = 0;
    debug_encoder_wheels_msg.data.data[1] = 0;
    debug_encoder_wheels_msg.data.data[2] = 0;
    debug_encoder_wheels_msg.data.data[3] = 0;
}

void cmd_reset_encoder_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    if (msg->linear.x > 0.5)
    {
        resetEncoderOffset();
    }
}

void publishData()
{
    debug_wheel_motor_msg.linear.x = cmd_vel_msg.linear.x;
    debug_wheel_motor_msg.linear.y = cmd_vel_msg.linear.y;
    debug_wheel_motor_msg.angular.z = cmd_vel_msg.angular.z;

    rcl_publish(&debug_cmd_vel_publisher, &debug_wheel_motor_msg, NULL);
    rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL);
    rcl_publish(&debug_encoder_wheels_publisher, &debug_encoder_wheels_msg, NULL);
}

void syncTime()
{
    // get the current time from the agent
        unsigned long now = millis();
        RCCHECK(rmw_uros_sync_session(10));
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
        tp.tv_sec = now / 1000;
        tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    ESP.restart();
    // while (true)
    // {
    //     delay(1000);
    // }
}