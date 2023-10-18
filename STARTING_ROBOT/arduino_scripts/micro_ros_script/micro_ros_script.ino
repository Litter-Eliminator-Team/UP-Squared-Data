#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>

#include <Adafruit_MotorShield.h>
#include <Encoder.h>
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Encoder knobLeft(1, 2);
Servo bucket_servo;
Servo sweeper_servo;
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher 1
rcl_publisher_t publisher_lin;
rclc_executor_t executor_pub_lin;

// publisher 2
rcl_publisher_t publisher_ang;
rclc_executor_t executor_pub_ang;

// publisher 3
rcl_publisher_t publisher_srv_one;
rclc_executor_t executor_srv_one;

// publisher 4
rcl_publisher_t publisher_srv_two;
rclc_executor_t executor_srv_two;

rcl_timer_t timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist motor_msg;
rclc_executor_t executor_sub;

//subscriber 2
rcl_subscription_t subscriber_servo;
std_msgs__msg__String servo_msg;
rclc_executor_t executor_sub_srv;
char test_array[200];

std_msgs__msg__Int32 servo_one_cmd;
std_msgs__msg__Int32 servo_two_cmd;

geometry_msgs__msg__Vector3 motorCommandLin;
geometry_msgs__msg__Vector3 motorCommandAng;

#define LED_PIN LED_BUILTIN

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher_lin, &motorCommandLin, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_ang, &motorCommandAng, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_srv_one, &servo_one_cmd, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_srv_two, &servo_two_cmd, NULL));

  }
}

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *motor_msg = (const geometry_msgs__msg__Twist *)msgin;
  motorCommandLin = motor_msg->linear;
  motorCommandAng = motor_msg->angular;

  // CIRCLE FORWARD CNTR CLK
  if(motorCommandLin.x == 0.5 && motorCommandAng.z == 1.0){
    rightMotor->setSpeed(70);
    rightMotor->run(FORWARD);

    leftMotor->setSpeed(0);
    leftMotor->run(RELEASE);
  // FORWARD
  }else if(motorCommandLin.x == 0.5 && motorCommandAng.z == 0.0){
    rightMotor->setSpeed(70);
    rightMotor->run(FORWARD);

    leftMotor->setSpeed(70);
    leftMotor->run(FORWARD);
  // CIRCLE FORWARD CLK
  }else if(motorCommandLin.x == 0.5 && motorCommandAng.z == -1.0){
    rightMotor->setSpeed(0);
    rightMotor->run(RELEASE);

    leftMotor->setSpeed(70);
    leftMotor->run(FORWARD);
  // TURN LEFT
  }else if(motorCommandLin.x == 0.0 && motorCommandAng.z == 1.0){
    rightMotor->setSpeed(70);
    rightMotor->run(FORWARD);

    leftMotor->setSpeed(70);
    leftMotor->run(BACKWARD);
  }else if(motorCommandLin.x == 0.0 && motorCommandAng.z == 0.0){
    rightMotor->setSpeed(0);
    rightMotor->run(BACKWARD);

    leftMotor->setSpeed(0);
    leftMotor->run(BACKWARD);
  }else if(motorCommandLin.x == 0.0 && motorCommandAng.z == -1.0){
    rightMotor->setSpeed(70);
    rightMotor->run(BACKWARD);

    leftMotor->setSpeed(70);
    leftMotor->run(FORWARD);
  }else if(motorCommandLin.x == -0.5 && motorCommandAng.z == -1.0){
    rightMotor->setSpeed(70);
    rightMotor->run(BACKWARD);

    leftMotor->setSpeed(0);
    leftMotor->run(RELEASE);
  }else if(motorCommandLin.x == -0.5 && motorCommandAng.z == 0.0){
    rightMotor->setSpeed(70);
    rightMotor->run(BACKWARD);

    leftMotor->setSpeed(70);
    leftMotor->run(BACKWARD);
  }else if(motorCommandLin.x == -0.5 && motorCommandAng.z == 1.0){
    rightMotor->setSpeed(0);
    rightMotor->run(RELEASE);

    leftMotor->setSpeed(70);
    leftMotor->run(BACKWARD);
  }
}

void subscription_callback_servo(const void *msgin)
{
  const std_msgs__msg__String *servo_msg = (const std_msgs__msg__String *)msgin;
  String servo_data(servo_msg->data.data);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(500);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(500);
  // Bucket Servo Forward
  if(servo_data == "w"){
    for(int i = servo_one_cmd.data; i < 240; i+=10){
      bucket_servo.write(i);
      delay(15);
    }
    servo_one_cmd.data = 240;
    // bucket_servo.write(servo_one_cmd.data);
    // delay(15); 
  // Bucket Servo Backward
  }else if(servo_data == "q" && servo_one_cmd.data == 240){
    for(int i = servo_one_cmd.data; i > -80; i-=10){
      bucket_servo.write(i);
      delay(15);
    }
    servo_one_cmd.data = -80;
    // bucket_servo.write(servo_one_cmd.data);
    // delay(15); 
  // Sweeper Servo Forward
  }else if(servo_data == "a"){
    servo_two_cmd.data = 150;
    sweeper_servo.write(servo_two_cmd.data);
    delay(15); 
  // Sweeper Servo Backward
  }else if(servo_data == "s"){
    servo_two_cmd.data = -90;
    sweeper_servo.write(servo_two_cmd.data);
    delay(15); 
  }

}

void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  bucket_servo.attach(6);
  sweeper_servo.attach(5);

  memset(test_array, 'z', 200);

  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // create subscriber 2
  RCCHECK(rclc_subscription_init_default(
      &subscriber_servo,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/glyphkey_pressed"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_lin,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "lin_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher_ang,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "ang_vel"));

  // create publisher 3
  RCCHECK(rclc_publisher_init_default(
      &publisher_srv_one,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_one_position"));

  // create publisher 4
  RCCHECK(rclc_publisher_init_default(
      &publisher_srv_two,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "servo_two_position"));

  // create timer, called every 1000 ms to publish heartbeat
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor publishers
  RCCHECK(rclc_executor_init(&executor_pub_lin, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub_ang, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_srv_one, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_srv_two, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor_pub_lin, &timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub_ang, &timer));
  RCCHECK(rclc_executor_add_timer(&executor_srv_one, &timer));
  RCCHECK(rclc_executor_add_timer(&executor_srv_two, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &motor_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_init(&executor_sub_srv, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_srv, &subscriber_servo, &servo_msg, &subscription_callback_servo, ON_NEW_DATA));

  servo_msg.data.data = (char * ) malloc(200 * sizeof(char));
	servo_msg.data.size = 0;
	servo_msg.data.capacity = 200;

}

void loop()
{
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_pub_lin, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub_ang, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_srv_one, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_srv_two, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub_srv, RCL_MS_TO_NS(100)));
}