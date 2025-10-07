#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <config.h>
#include <motor.h>
#include <ESP32Servo.h>

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

// -------------------- Motor Controller --------------------
Controller motorload(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTORLOAD_INV, MOTORLOAD_BRAKE, MOTORLOAD_PWM, MOTORLOAD_IN_A, MOTORLOAD_IN_B);

void load(int MotorloadSpeed)
{
    MotorloadSpeed = constrain(MotorloadSpeed, PWM_Min, PWM_Max);
    motorload.spin(MotorloadSpeed);
}


// -------------------- Servo Controller --------------------
Servo servo_dig;

void controlServo(float angle)
{
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    servo_dig.write(angle);
}

// -------------------- Callback --------------------

void cmd_load_callback(const void *msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    load(msg->linear.x);
}

void cmd_servo_callback(const void * msgin)
{
     const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    controlServo(msg->angular.x);
}

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "load_node", "", &support);

    // Publisher
    rclc_publisher_init_default(
        &debug_load_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/load");

    rclc_publisher_init_default(
        &debug_servo_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/servo");
    
    // Subscription
    rclc_subscription_init_default(
        &cmd_load_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/teelek/cmd_load");

    rclc_subscription_init_default(
        &cmd_servo_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/teelek/cmd_servo");

    // --- Executor ---
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 4, &allocator);

    rclc_executor_add_subscription(&executor, 
        &cmd_load_subscriber, 
        &cmd_load_msg, 
        &cmd_load_callback, 
        ON_NEW_DATA);

    rclc_executor_add_subscription(&executor, 
        &cmd_servo_subscriber, 
        &cmd_servo_msg, 
        &cmd_servo_callback, 
        ON_NEW_DATA);
    
    // --- Servo init ---
    servo_dig.attach(SERVO_PIN);
    servo_dig.write(0);  // เริ่มที่มุมกลาง
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&cmd_load_subscriber, &node);
    rcl_subscription_fini(&cmd_servo_subscriber, &node);
    rcl_publisher_fini(&debug_load_publisher, &node);
    rcl_publisher_fini(&debug_servo_publisher, &node);

    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void publishData()
{
    debug_load_msg.linear.x = cmd_load_msg.linear.x;
    rcl_publish(&debug_load_publisher, &debug_load_msg, NULL);

    debug_servo_msg.angular.x = cmd_servo_msg.angular.x;
    rcl_publish(&debug_servo_publisher, &debug_servo_msg, NULL);
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
