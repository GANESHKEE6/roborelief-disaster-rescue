#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <Wire.h>
#include <AS5600.h>
#include <ESP32Servo.h>

/* ===================== CONFIG ===================== */

// WiFi credentials
char ssid[] = "PRABHU";
char password[] = "kee129..";

// micro-ROS Agent (PC running micro-ROS agent)
IPAddress agent_ip(10,239,213,239); 
uint16_t agent_port = 8888;

/* ===================== HARDWARE ===================== */

AS5600 as5600;
Servo smokeServo;

// Pins
#define MQ2_PIN    34
#define SERVO_PIN  18
#define LED_PIN    35

/* =================== MICRO-ROS =================== */

rcl_publisher_t pub_smoke;
std_msgs__msg__Int32 msg_smoke;

rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

/* ===================== MACROS ===================== */

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void) rc; }

/* ===================== ERROR LOOP ===================== */

void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

/* ===================== TIMER CALLBACK ===================== */

// This now runs incredibly fast: every 50ms (20 times a second)
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;

  if (timer != NULL)
  {
    // -----------------------------
    // 1. Encoder -> Servo Logic (HIGH SENSITIVITY)
    // -----------------------------
    float towerAngle = as5600.readAngle() * 360.0 / 4096.0;
    float servoAngle = 0;

    // Cut off excessive readings from 180 to 360
    if(towerAngle > 180.0) {
      servoAngle = 180.0;
    } else {
      servoAngle = towerAngle;
    }
    
    smokeServo.write(servoAngle);

    // -----------------------------
    // 2. MQ2 -> LED Logic
    // -----------------------------
    int smokeValue = analogRead(MQ2_PIN);
    
    if(smokeValue < 1200) {
      digitalWrite(LED_PIN, LOW); // SAFE
    } 
    else if(smokeValue < 2200) {
      digitalWrite(LED_PIN, LOW); // LOW RISK
    } 
    else if(smokeValue < 3200) {
      digitalWrite(LED_PIN, LOW); // MEDIUM RISK
    } 
    else {
      digitalWrite(LED_PIN, HIGH); // HIGH RISK
    }

    // -----------------------------
    // 3. Publish MQ2 to ROS (Safeguard)
    // -----------------------------
    // We use a counter so we don't spam the WiFi network 20 times a second.
    // It will publish every 4th tick (4 * 50ms = 200ms).
    static int publish_counter = 0;
    publish_counter++;
    
    if (publish_counter >= 4) {
      msg_smoke.data = smokeValue;
      RCSOFTCHECK(rcl_publish(&pub_smoke, &msg_smoke, NULL));
      publish_counter = 0; // Reset counter
    }
  }
}

/* ===================== SETUP ===================== */

void setup()
{
  Serial.begin(115200);
  
  // Setup Hardware
  Wire.begin(21, 22);
  smokeServo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);
  
  analogReadResolution(12);
  pinMode(MQ2_PIN, INPUT);

  delay(2000);

  // Configure WiFi micro-ROS transport
  set_microros_wifi_transports(
    ssid,
    password,
    agent_ip,
    agent_port
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(
    rclc_node_init_default(
      &node,
      "esp32_forest_fire_node",
      "",
      &support
    )
  );

  // Create Publisher
  RCCHECK(
    rclc_publisher_init_default(
      &pub_smoke,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "sensor/mq2_smoke"
    )
  );

  // Create timer (20 Hz / 50ms interval for high sensitivity)
  const unsigned int timer_timeout = 50;
  RCCHECK(
    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback
    )
  );

  // Create executor 
  RCCHECK(
    rclc_executor_init(
      &executor,
      &support.context,
      1, 
      &allocator
    )
  );

  // Add timer handle to executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

/* ===================== LOOP ===================== */

void loop()
{
  // Spin the executor to handle networking
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(10) // Lowered spin timeout to match the faster hardware loop
    )
  );

  delay(5); // Shorter delay so the loop runs fast enough for the 50ms timer
}