#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

const int Sensor_Fuerza = 34;
#define LED_BUILTIN 2 

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

// Error macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setup() {

  set_microros_transports();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Sensor_Fuerza, INPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "force_sensor_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "force_sensor"));

}

void loop() {

  long suma = 0;

  // Promedio de 10 lecturas
  for(int i = 0; i < 10; i++){
    suma += analogRead(Sensor_Fuerza);
    delay(2);
  }

  int valorFuerza = suma / 10;

  msg.data = valorFuerza;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  delay(100); // ~10 Hz
}