#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

/* ===================== CONFIG ===================== */
char ssid[] = "PRABHU";
char password[] = "kee129..";

// Raspberry Pi IP
IPAddress agent_ip(10,239,213,239);
uint16_t agent_port = 8888;

MPU6050 mpu(Wire);

/* ===================== ROS OBJECTS ===================== */
// Publishers
rcl_publisher_t pub_tilt;
rcl_publisher_t pub_vib;
rcl_publisher_t pub_status;

// Messages
geometry_msgs__msg__Vector3 tilt_msg;
std_msgs__msg__Float32 vib_msg;
std_msgs__msg__String status_msg;

rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Variable to track highest vibration between timer ticks
float max_vibration_g = 0.0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void) rc; }

void error_loop() {
  while (1) {
    digitalWrite(2, !digitalRead(2)); 
    delay(100);
  }
}

/* ===================== TIMER CALLBACK ===================== */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    
    // 1. Get Angles & Calculate Tilt
    float pitch = mpu.getAngleX();
    float roll  = mpu.getAngleY();
    float tilt  = sqrt(pitch * pitch + roll * roll);

    // 2. Get Vibration and Reset Tracker
    float current_vib = max_vibration_g;
    max_vibration_g = 0.0; 

    // 3. Determine Status
    const char* status_str;
    if (tilt >= 18 || current_vib >= 0.3) {
      status_str = "DANGER";
    } 
    else if ((tilt >= 10 && tilt < 18) || current_vib >= 0.1) {
      status_str = "WARNING";
    } 
    else {
      status_str = "SAFE";
    }

    // --- PUBLISH DATA ---

    // Publish Tilt (Vector3)
    tilt_msg.x = pitch;
    tilt_msg.y = roll;
    tilt_msg.z = tilt;
    RCSOFTCHECK(rcl_publish(&pub_tilt, &tilt_msg, NULL));

    // Publish Vibration (Float32)
    vib_msg.data = current_vib;
    RCSOFTCHECK(rcl_publish(&pub_vib, &vib_msg, NULL));

    // Publish Status (String)
    snprintf(status_msg.data.data, status_msg.data.capacity, "%s", status_str);
    status_msg.data.size = strlen(status_msg.data.data);
    RCSOFTCHECK(rcl_publish(&pub_status, &status_msg, NULL));

    // --- SERIAL DEBUGGING ---
    Serial.printf("Pitch: %.2f | Roll: %.2f | Tilt: %.2f | Vib: %.3fg | Status: %s\n", 
                  pitch, roll, tilt, current_vib, status_str);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Initialize I2C and MPU6050
  Wire.begin(21, 22);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  if(status != 0) { 
    Serial.println("MPU6050 still not found!");
    error_loop(); 
  }

  Serial.println(F("Calculating offsets, keep sensor steady..."));
  delay(1000);
  mpu.calcOffsets(); 
  Serial.println(F("Calibration Done!\n"));

  // WiFi & micro-ROS Setup
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "landslide_monitor_node", "", &support));

  // Init Tilt Publisher (Vector3)
  RCCHECK(rclc_publisher_init_default(
    &pub_tilt, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), 
    "landslide/tilt"));

  // Init Vibration Publisher (Float32)
  RCCHECK(rclc_publisher_init_default(
    &pub_vib, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
    "landslide/vibration"));

  // Init Status Publisher (String)
  RCCHECK(rclc_publisher_init_default(
    &pub_status, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
    "landslide/status"));

  // Allocate memory for the string
  status_msg.data.data = (char *) malloc(20 * sizeof(char));
  status_msg.data.capacity = 20;

  // Set Timer to 200ms (5 Hz)
  const unsigned int timer_timeout = 200; 
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  Serial.println("Ready to Publish to 3 distinct ROS 2 topics!");
}

void loop() {
  // Update the MPU as fast as possible for accurate filtering
  mpu.update();

  // --- VIBRATION CALCULATION ---
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  
  float accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
  float instant_vibration = abs(accel_magnitude - 1.0);

  if (instant_vibration > max_vibration_g) {
    max_vibration_g = instant_vibration;
  }

  // Spin the executor briefly to handle ROS tasks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(2);
}