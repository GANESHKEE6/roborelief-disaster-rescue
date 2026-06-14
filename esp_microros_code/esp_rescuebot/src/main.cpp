#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point.h>
#include <std_msgs/msg/string.h>

/* ===================== PIN CONFIG (TB6612FNG) ===================== */
#define PWMA 32
#define AIN1 25
#define AIN2 26
#define BIN1 27
#define BIN2 14
#define PWMB 33
#define STBY 4

#define L_ENC 18
#define R_ENC 19

/* ===================== CONSTANTS ===================== */
const float DIST_PER_TICK = 0.0102; // 1.02cm in meters
const int MOVE_SPEED = 160;
const int TURN_SPEED = 175;

/* ===================== GLOBALS ===================== */
MPU6050 mpu(Wire);
volatile long l_pulses = 0, r_pulses = 0;
float current_x = 0.0, current_y = 0.0;

// Micro-ROS objects
rcl_subscription_t goal_sub;
rcl_publisher_t status_pub;
geometry_msgs__msg__Point goal_msg;
std_msgs__msg__String status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return; }

/* ===================== HELPERS ===================== */
void IRAM_ATTR l_isr() { l_pulses++; }
void IRAM_ATTR r_isr() { r_pulses++; }

void drive(int l, int r) {
    // Left Motor (Motor A)
    digitalWrite(AIN1, l >= 0 ? HIGH : LOW); 
    digitalWrite(AIN2, l >= 0 ? LOW : HIGH);
    analogWrite(PWMA, abs(l));
    
    // Right Motor (Motor B)
    digitalWrite(BIN1, r >= 0 ? HIGH : LOW); 
    digitalWrite(BIN2, r >= 0 ? LOW : HIGH);
    analogWrite(PWMB, abs(r));
}

void stopBot() { drive(0, 0); delay(1000); }

void send_status(const char* text) {
    status_msg.data.data = (char*)text;
    status_msg.data.size = strlen(text);
    rcl_publish(&status_pub, &status_msg, NULL);
}

/* ===================== NAVIGATION LOGIC ===================== */

void turnToAbsolute(float target_yaw) {
    while (true) {
        mpu.update();
        float error = target_yaw - mpu.getAngleZ();
        if (abs(error) < 3.0) break;

        int speed = (abs(error) > 20) ? TURN_SPEED : 145;
        if (error < 0) drive(speed, -speed); // CW
        else drive(-speed, speed);           // CCW

        delay(40); drive(0,0); delay(10); // Noise suppression logic
    }
    stopBot();
}

void moveForwardMeters(float meters) {
    long target_ticks = meters / DIST_PER_TICK;
    l_pulses = 0; r_pulses = 0;
    while ((l_pulses + r_pulses) / 2 < target_ticks) {
        drive(MOVE_SPEED, MOVE_SPEED);
        delay(10);
    }
    stopBot();
}

/* ===================== CALLBACK ===================== */

void goal_callback(const void * msgin) {
    const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
    
    float dx = msg->x - current_x;
    float dy = msg->y - current_y;
    float distance = sqrt(dx*dx + dy*dy);
    float target_angle = atan2(dy, dx) * 180.0 / PI;

    send_status("Calculating Path...");
    
    // 1. Turn to Goal
    turnToAbsolute(target_angle);
    
    // 2. Move to Goal
    moveForwardMeters(distance);
    
    // 3. Update Position
    current_x = msg->x;
    current_y = msg->y;
    
    send_status("Goal Reached!");
}

/* ===================== SETUP & LOOP ===================== */

void setup() {
    Wire.begin();
    Serial.begin(115200);
    
    // Initialize TB6612FNG Pins
    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    
    // Activate the motor driver
    digitalWrite(STBY, HIGH);

    // Initialize Encoders
    pinMode(L_ENC, INPUT_PULLUP); attachInterrupt(L_ENC, l_isr, RISING);
    pinMode(R_ENC, INPUT_PULLUP); attachInterrupt(R_ENC, r_isr, RISING);

    // Initialize MPU6050
    mpu.begin();
    delay(2000); 
    mpu.calcOffsets();

    // Initialize micro-ROS
    set_microros_wifi_transports((char*)"PRABHU", (char*)"kee129..", IPAddress(10, 52, 106, 239), 8888);
    delay(2000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "rescue_bot", "", &support));

    RCCHECK(rclc_publisher_init_default(&status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/bot/status"));
    RCCHECK(rclc_subscription_init_default(&goal_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/bot/goal"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &goal_sub, &goal_msg, &goal_callback, ON_NEW_DATA));

    status_msg.data.capacity = 50;
    status_msg.data.data = (char*) malloc(status_msg.data.capacity);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}