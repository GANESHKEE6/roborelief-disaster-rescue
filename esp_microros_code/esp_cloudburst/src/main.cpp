#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for disabling power saving
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <DHT.h>

/* ===================== CONFIG ===================== */

// WiFi credentials
char ssid[] = "PRABHU";
char password[] = "kee129..";

// micro-ROS Agent (PC running micro-ROS agent)
IPAddress agent_ip(10, 239, 213, 239); // Corrected to match your working boards
uint16_t agent_port = 8888;

// DHT11 sensor pin
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/* ================================================== */

rcl_publisher_t temp_pub;
rcl_publisher_t hum_pub;

std_msgs__msg__Float32 temp_msg;
std_msgs__msg__Float32 hum_msg;

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
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    temp_msg.data = temperature;
    hum_msg.data = humidity;

    RCSOFTCHECK(rcl_publish(&temp_pub, &temp_msg, NULL));
    RCSOFTCHECK(rcl_publish(&hum_pub, &hum_msg, NULL));

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }
}

/* ===================== SETUP ===================== */

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Initialize DHT sensor
  dht.begin();

  // Configure WiFi micro-ROS transport
  set_microros_wifi_transports(
    ssid,
    password,
    agent_ip,
    agent_port
  );

  delay(2000);

  // Force Wi-Fi to stay awake for multi-node setups to prevent UDP packet drops
  esp_wifi_set_ps(WIFI_PS_NONE);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(
    rclc_node_init_default(
      &node,
      "dht11_node",
      "",
      &support
    )
  );

  // Create publishers
  RCCHECK(
    rclc_publisher_init_default(
      &temp_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "dht11/temperature"
    )
  );

  RCCHECK(
    rclc_publisher_init_default(
      &hum_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "dht11/humidity"
    )
  );

  // Create timer (1 Hz)
  const unsigned int timer_timeout = 500;
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