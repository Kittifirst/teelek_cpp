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
#include <Utilize.h>
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

// -------------------- ROS Entities --------------------
rcl_subscription_t cmd_load_subscriber;
geometry_msgs__msg__Twist cmd_load_msg;

rcl_publisher_t debug_load_publisher;
geometry_msgs__msg__Twist debug_load_msg;

rcl_subscription_t cmd_servo_subscriber;
geometry_msgs__msg__Twist cmd_servo_msg;

rcl_publisher_t debug_servo_publisher;
geometry_msgs__msg__Twist debug_servo_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
static unsigned long last_sync = 0;

// -------------------- Function Prototypes --------------------
void load(int MotorloadSpeed);
void controlServo(float pulse);
void cmd_load_callback(const void *msgin);
void cmd_servo_callback(const void *msgin);
void publishData();

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

bool createEntities();
bool destroyEntities();

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// -------------------- Servo --------------------
Controller motorload(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTORLOAD_INV, MOTORLOAD_BRAKE, MOTORLOAD_PWM, MOTORLOAD_IN_A, MOTORLOAD_IN_B);
Servo servo_dig;

// -------------------- Setup / Loop --------------------
void setup() {
    servo_dig.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    servo_dig.writeMicroseconds(900);

    Serial.begin(115200);
    #ifdef MICROROS_WIFI
        IPAddress agent_ip(AGENT_IP);
        uint16_t agent_port = AGENT_PORT;
        set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
    #else
        set_microros_serial_transports(Serial);
    #endif

    state = WAITING_AGENT;
    last_sync = millis();
}

void loop() {
    // Sync time every 60 seconds
    if (millis() - last_sync > 60000) {
        syncTime();
        last_sync = millis();
    }

    switch (state) {
        case WAITING_AGENT:
            // Ping agent every 1 second
            EXECUTE_EVERY_N_MS(1000, {
                if (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) {
                    state = AGENT_AVAILABLE;
                }
            });
            break;

        case AGENT_AVAILABLE:
            static unsigned long last_attempt = 0;
            if (millis() - last_attempt > 2000) {
                last_attempt = millis();
                if (createEntities()) {
                    state = AGENT_CONNECTED;
                }
            }
            break;

        case AGENT_CONNECTED:
            // Spin executor
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

            // Check agent connection every 2 seconds
            EXECUTE_EVERY_N_MS(2000, {
                if (!(RMW_RET_OK == rmw_uros_ping_agent(1000, 1))) {
                    state = AGENT_DISCONNECTED;
                }
            });
            break;

        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;

        default:
            break;
    }
}

// -------------------- Function --------------------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
            publishData(); 
        }
    }

void load(int MotorloadSpeed) {
        MotorloadSpeed = constrain(MotorloadSpeed, PWM_Min, PWM_Max);
        motorload.spin(MotorloadSpeed);
    }

void controlServo(float pulse) {
        pulse = constrain(pulse, 800, 3200);
        servo_dig.writeMicroseconds(pulse);
    }

    // -------------------- Callbacks --------------------
void cmd_load_callback(const void *msgin) {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        cmd_load_msg = *msg;
        load(msg->linear.x);
    }

void cmd_servo_callback(const void *msgin) {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        cmd_servo_msg = *msg;
        controlServo(msg->angular.x);
    }

// -------------------- Create Entities Safe --------------------
bool createEntities()
{
    allocator = rcl_get_default_allocator();
    
    init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
    rcl_init_options_set_domain_id(&init_options, 10);

    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;

    // create node
    #ifdef teelek_karake
        if (rclc_node_init_default(&node, "teelek_karake", "", &support) != RCL_RET_OK) return false;
    #elif teelek_katsu
        if (rclc_node_init_default(&node, "teelek_katsu", "", &support) != RCL_RET_OK) return false;
    #endif

    if (rclc_publisher_init_default(&debug_load_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/teelek/debug/load") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&debug_servo_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/teelek/debug/servo") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&cmd_load_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/teelek/cmd_load") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&cmd_servo_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/teelek/cmd_servo") != RCL_RET_OK) return false;

    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 5, &allocator) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(&executor, &cmd_load_subscriber, &cmd_load_msg,
        &cmd_load_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(&executor, &cmd_servo_subscriber, &cmd_servo_msg,
        &cmd_servo_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    const unsigned int control_timeout = 50;
    if (rclc_timer_init_default(&control_timer, &support,
        RCL_MS_TO_NS(control_timeout), &controlCallback) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;

    syncTime();
    return true;
}

// -------------------- Destroy Entities --------------------
bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&cmd_servo_subscriber, &node);
    rcl_subscription_fini(&cmd_load_subscriber, &node);
    rcl_publisher_fini(&debug_load_publisher, &node);
    rcl_publisher_fini(&debug_servo_publisher, &node);

    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void publishData()
{
    debug_load_msg.linear.x = cmd_load_msg.linear.x;
    debug_servo_msg.angular.x = cmd_servo_msg.angular.x;

    rcl_publish(&debug_load_publisher, &debug_load_msg, NULL);
    rcl_publish(&debug_servo_publisher, &debug_servo_msg, NULL);
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
