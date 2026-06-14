#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <Wire.h>
#include <MPU6050_light.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

/* ===================== CONFIG ===================== */

// WiFi credentials
char ssid[] = "PRABHU";
char password[] = "kee129..";

// micro-ROS Agent (PC running micro-ROS agent)
IPAddress agent_ip(10,239,213,239);   // CHANGE THIS
uint16_t agent_port = 8888;

// MPU6050 Sensor
MPU6050 mpu(Wire);

// Gravity estimate
float gx = 0;
float gy = 0;
float gz = 0;

/* ================================================== */

rcl_publisher_t vibration_pub;
rcl_publisher_t risk_pub;
rcl_publisher_t intensity_pub;

std_msgs__msg__Float32 vibration_msg;
std_msgs__msg__Float32 risk_msg;
std_msgs__msg__String intensity_msg;

// Buffer to hold the intensity string for micro-ROS
char intensity_buffer[20];

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

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;

  if (timer != NULL)
  {
    float vibrationSum = 0;

    // Average 10 samples
    for (int i = 0; i < 10; i++)
    {
      mpu.update();

      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();

      // Low-pass filter for gravity
      gx = 0.995 * gx + 0.005 * ax;
      gy = 0.995 * gy + 0.005 * ay;
      gz = 0.995 * gz + 0.005 * az;

      // Remove gravity
      float vx = ax - gx;
      float vy = ay - gy;
      float vz = az - gz;

      float instantVibration = sqrt(vx * vx + vy * vy + vz * vz);

      vibrationSum += instantVibration;

      delay(1);
    }

    // Average vibration level
    float vibration = vibrationSum / 10.0;

    // Calculate Risk Score (0 - 100)
    float riskScore = constrain(vibration * 500, 0.0, 100.0);

    // Intensity Classification
    if (vibration < 0.015)
    {
      strcpy(intensity_buffer, "SAFE");
    }
    else if (vibration < 0.03)
    {
      strcpy(intensity_buffer, "MILD TREMOR");
    }
    else if (vibration < 0.07)
    {
      strcpy(intensity_buffer, "MODERATE");
    }
    else if (vibration < 0.15)
    {
      strcpy(intensity_buffer, "SEVERE");
    }
    else
    {
      strcpy(intensity_buffer, "CRITICAL");
    }

    // Assign values to ROS messages
    vibration_msg.data = vibration;
    risk_msg.data = riskScore;
    
    // Assign string buffer to string message
    intensity_msg.data.data = intensity_buffer;
    intensity_msg.data.size = strlen(intensity_buffer);
    intensity_msg.data.capacity = sizeof(intensity_buffer);

    // Publish messages
    RCSOFTCHECK(rcl_publish(&vibration_pub, &vibration_msg, NULL));
    RCSOFTCHECK(rcl_publish(&risk_pub, &risk_msg, NULL));
    RCSOFTCHECK(rcl_publish(&intensity_pub, &intensity_msg, NULL));

    // Serial Debug Output
    Serial.print("Vibration: ");
    Serial.print(vibration, 5);
    Serial.print(" | Risk Score: ");
    Serial.print(riskScore);
    Serial.print(" | Intensity: ");
    Serial.println(intensity_buffer);
  }
}

/* ===================== SETUP ===================== */

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Initialize MPU6050 sensor
  Wire.begin(21, 22);
  byte status = mpu.begin();

  if (status != 0)
  {
    Serial.print("MPU6050 Error: ");
    Serial.println(status);
    error_loop();
  }

  Serial.println("Keep sensor stationary...");
  delay(2000);

  mpu.calcOffsets(true, true);
  Serial.println("Calibration Complete");

  // Initial gravity estimate
  for (int i = 0; i < 200; i++)
  {
    mpu.update();
    gx += mpu.getAccX();
    gy += mpu.getAccY();
    gz += mpu.getAccZ();
    delay(5);
  }

  gx /= 200.0;
  gy /= 200.0;
  gz /= 200.0;

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
      "seismometer_node",
      "",
      &support
    )
  );

  // Create publishers
  RCCHECK(
    rclc_publisher_init_default(
      &vibration_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "seismometer/vibration"
    )
  );

  RCCHECK(
    rclc_publisher_init_default(
      &risk_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "seismometer/risk_score"
    )
  );

  RCCHECK(
    rclc_publisher_init_default(
      &intensity_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "seismometer/intensity"
    )
  );

  // Create timer (50 Hz / 20ms to align with the original loop delay timing)
  const unsigned int timer_timeout = 20;
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

  RCCHECK(
    rclc_executor_add_timer(&executor, &timer)
  );
}

/* ===================== LOOP ===================== */

void loop()
{
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(100)
    )
  );

  delay(10);
}