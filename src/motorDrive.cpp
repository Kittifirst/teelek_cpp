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

#include <esp32_Encoder.h>    
#include <ESP32Servo.h>

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

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

long long ticks_L_front = 0;
long long ticks_R_front = 0;

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

//------------------------------ < Fuction Prototype > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

void publishData();
void getEncoderData();
void MovePower(int, int, int, int);
//------------------------------ < Main > -------------------------------------//

void setup()
{

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
    switch (state)
    {
    case WAITING_AGENT:
        // EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        EXECUTE_EVERY_N_MS(1500, state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 10)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(700, state = (RMW_RET_OK == rmw_uros_ping_agent(600, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
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


void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        getEncoderData();
        publishData();
    }
}

void cmd_vel_callback(const void * msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float V_x = msg->linear.x;
    float V_y = 0.0;
    float W_z = msg->angular.z;
    float d = max(abs(V_x) + abs(V_y) + abs(W_z), (float) PWM_Max);
    int fl = (V_x - W_z)/d * (float) PWM_Max;
    int fr = (V_x + W_z)/d * (float) PWM_Max;
    int bl = (V_x - W_z)/d * (float) PWM_Max;
    int br = (V_x + W_z)/d * (float) PWM_Max;
    MovePower(fl, fr,
              bl, br);
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
        
    // create timer for control loop 1000/40 Hz
    const unsigned int control_timeout = 40;
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

    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_subscription_fini(&cmd_vel_subscriber, &node);

    rcl_publisher_fini(&debug_cmd_vel_publisher, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void getEncoderData()
{
    // #ifdef teelek_karake
    // // Get encoder data
    // rpm_front_L = Encoder1.getRPM();
    // rpm_front_R = Encoder2.getRPM();
    // rpm_rear_left_L = Encoder3.getRPM();
    // rpm_rear_left_R = Encoder4.getRPM();

    // // debug_wheel_motorRPM_msg.linear.x = rpm_front_L;
    // // debug_wheel_motorRPM_msg.linear.y = rpm_front_R;
    // // debug_wheel_motorRPM_msg.linear.z = rpm_rear_left_L;
    // // debug_wheel_motorRPM_msg.angular.x = rpm_rear_left_R;
    // #elif teelek_katsu
    // rpm_rear_right_L = Encoder5.getRPM();
    // rpm_rear_right_R = Encoder6.getRPM();

    // // debug_wheel_motorRPM_msg.angular.y = rpm_rear_right_L;
    // // debug_wheel_motorRPM_msg.angular.z = rpm_rear_right_R;
    // #endif

    // // debug_wheel_encoder_msg.linear.x = 0.0;
    // // debug_wheel_encoder_msg.linear.y = 0.0;
    // // debug_wheel_encoder_msg.linear.z = 0.0;
    // // debug_wheel_encoder_msg.linear.z = Encoder3.getRPM();

}

void publishData()
{
    debug_wheel_motor_msg.linear.x = cmd_vel_msg.linear.x;
    debug_wheel_motor_msg.linear.y = cmd_vel_msg.linear.y;
    debug_wheel_motor_msg.angular.z = cmd_vel_msg.angular.z;
    struct timespec time_stamp = getTime();
    rcl_publish(&debug_cmd_vel_publisher, &debug_wheel_motor_msg, NULL);
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