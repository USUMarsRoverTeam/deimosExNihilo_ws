#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int16_multi_array.h>

#include <Servo.h>
#include <PolouMaestro.h>

#define maestroSerial Serial1
MicroMaestro maestro(maestroSerial);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Subscriber Nodes
rcl_subscription_t sub_twist;   // Drive Motor Controls
rcl_subscription_t sub_arm;     // Arm Controls
rcl_subscription_t sub_science; // Auger & Plunger Control
rcl_subscription_t sub_syringe; // Syringe Motor Control
rcl_subscription_t sub_carousel;// Carousel Motor Control
rcl_subscription_t sub_camera;  // Camera Servo Control

// Publishers 
rcl_publisher_t pub_temp;
rcl_publisher_t pub_humidity;
rcl_publisher_t pub_arm_pos;
/*Potential Other Publishers:
  - Odometry?
  - Heading? If GPS Through Arduino
  - Linear Actuator Position*/

// Messages
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__UInt16MultiArray arm_msg, camera_msg;
std_msgs__msg__Float32MultiArray science_msg;
std_msgs__msg__Int32 syringe_msg, carousel_msg;
std_msgs__msg__Float32 temp_msg, humidity_msg;

// Pinout
#define PANO_CAM_SERVO_PIN 5
#define LEFT_DRIVE_PIN 6
#define RIGHT_DRIVE_PIN 7
#define SYRINGE_STEP_PIN 8
#define SYRINGE_DIR_PIN 9
#define AUGER_PIN 10
#define PLUNGER_PIN 11
#define CAROUSEL_STEP_PIN 12
#define CAROUSEL_DIR_PIN 13
#define TEMP_PIN A0
#define HUMIDITY_PIN A1

// System Parameters
const int stepperDelay = 100; // Corresponds to 5k Steps/Second
const float tempSlope = 0.04;
const int tempIntercept = -40;
const float humiditySlope = 0.04;
const float humidityIntercept = 0.0;
double wheelBase = 0.92;
double center2edge = wheelBase / 2.0;
double maxLinearVelocity = 0.5;
double maxSpinVelocity = 2.0;
double maxWheelVelocity = maxLinearVelocity + maxSpinVelocity * center2edge;
int pwmVals[3] = [1000, 1500, 2000];
float deadzone = 0.05;
unsigned long lastCmdTime = 0;
unsigned long cmdTimeout = 300;

// Object Initialization
Servo leftDrive, rightDrive, augerMotor, plungerMotor, panoCamServo;

// Error Handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Callback Functions For Evaluating Controls

// Drivetrain Motor Controls
void twistCallback(const void * msgin){
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  lastCmdTime = millis();

  // Dead Zone handling
  double linVel = (abs(msg->linear.x) < deadzone) ? 0.0 : msg->linear.x;
  double spinVel = (abs(msg->angular.z) < deadzone) ? 0.0 : msg->angular.z;

  // Constraining Velocity
  linVel = constrain(linVel, -maxLinearVelocity, maxLinearVelocity);
  spinVel = constrain(spinVel, -maxSpinVelocity, maxSpinVelocity);

  // Splitting into Left and Right Motor Velocities
  double leftDriveVel = linVel - spinVel * center2edge;
  double rightDriveVel = linVel + spinVel * center2edge;

  setDrivePWM(leftDriveVel, rightDriveVel);
}

void armCallback(const void * msgin){
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  for (int i = 0; i < 6 && i < msg->data.size; i++){
    maestro.setTarget(i, msg->data.data[i]);
  }
}

void scienceModuleCallback(const void * msgin){
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size == 2){
    augerMotor.writeMicroseconds(constrain(msg->data.data[0], 1000, 2000));
    plungerMotor.writeMicroseconds(constrain(msg->data.data[1], 1000, 2000));
  }
}

void syringeStepCallback(const void * msgin){
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  moveStepperSteps(SYRINGE_STEP_PIN, SYRINGE_DIR_PIN, msg->data)
}

void carouselStepCallback(const void * msgin){
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  moveStepperSteps(CAROUSEL_STEP_PIN, CAROUSEL_DIR_PIN, msg->data)
}

void panoCamServoCallback(const void * msgin){
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  if (msg->data.size){
    panoCamServo.write(msg->data.data[0]);
  }
}

// Helper Functions
void setDrivePWM(double left, double right){
  // Constrain Wheel Velocity
  left = constrain(left, -maxWheelVelocity, maxWheelVelocity);
  right = constrain(right, -maxWheelVelocity, maxWheelVelocity);

  // Map to PWM Scale
  int leftPWM = map(left, -maxWheelVelocity, maxWheelVelocity, pwmVals[0], pwmVals[2]);
  int rightPWM = 2 * pwmVals[1] - map(right, -maxWheelVelocity, maxWheelVelocity, pwmVals[0], pwmVals[2]);
  
  leftDrive.writeMicroseconds(leftPWM);
  rightDrive.writeMicroseconds(rightPWM);
}

void setDriveNeutralPWM() {
  leftDrive.writeMicroseconds(pwmVals[1]);
  rightDrive.writeMicroseconds(pwmVals[1]);
}

void moveStepper(int stepPin, int dirPin, int stepCount){
  // Set Direction
  digitalWrite(dirPin, stepCount >= 0 ? HIGH : LOW);

  // Create Square Wave Control Signal 
  for (int i = 0; i < abs(stepCount); i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepperDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepperDelay)
  }
}

float [] calcTempAndHumidity(int tempRaw, int humidityRaw){
  // Calculating Temperature
  float tempVoltage = tempRaw * (3.3/1023.0);
  float tempC = (tempVoltage * 1000) * tempSlope + tempIntercept;

  // Calculating Humidity
  float humidityVoltage = humidityRaw * (3.3/1023.0);
  float humidity = (humidityVoltage * 1000) * humiditySlope + humidityIntercept;
  return [tempC, humidity];
}

// Needed for Arduino 
void setup(){

  // Begin Serial Ports
  Serial.begin(115200);
  maestroSerial.begin(9600);

  // Attach Motors to Pins
  leftDrive.attach(LEFT_DRIVE_PIN);
  rightDrive.attach(RIGHT_DRIVE_PIN);
  setDriveNeutralPWM() 
  
  augerMotor.attach(AUGER_PIN);
  plungerMotor.attach(PLUNGER_PIN);

  panoCamServo.attach(PANO_CAM_SERVO_PIN);
  panoCamServo.write(67);

  // Attach Stepper Pins
  pinMode(SYRINGE_STEP_PIN, OUTPUT);
  pinMode(SYRINGE_DIR_PIN, OUTPUT);
  pinMode(CAROUSEL_STEP_PIN, OUTPUT);
  pinMode(CAROUSEL_DIR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  set_microros_transports();
  delay(2000);

  // Creating init_options
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_int(&support, 0, NULL, &allocator));

  // Create ROS Node
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

    // Create subscribers
  RCCHECK(rclc_subscription_init_default(
    &sub_twist,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &sub_servo,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "servo_positions"));

  RCCHECK(rclc_subscription_init_default(
    &sub_science,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "science_module"));

  RCCHECK(rclc_subscription_init_default(
    &sub_syringe,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "syringe_count"));

  RCCHECK(rclc_subscription_init_default(
    &sub_carousel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "carousel_count"));

  RCCHECK(rclc_subscription_init_default(
    &sub_camera,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "camera_servo_angle"));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &pub_temp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "temperature"));

  RCCHECK(rclc_publisher_init_default(
    &pub_humidity,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "humidity"));

  // Create executor
  const int num_handles = 6;  // 6 subscribers
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  
  // Create Subscriptions for the Subscribers
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_twist, &twist_msg, &twistCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo, &servo_msg, &servoCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_science, &science_msg, &scienceModuleCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_syringe, &syringe_msg, &syringeStepCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_carousel, &carousel_msg, &carouselStepCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_camera, &camera_msg, &cameraServoCallback, ON_NEW_DATA));

  // Allocate memory for dynamic arrays
  servo_msg.data.capacity = 6;
  servo_msg.data.size = 0;
  servo_msg.data.data = (uint16_t*) malloc(servo_msg.data.capacity * sizeof(uint16_t));
  
  science_msg.data.capacity = 2;
  science_msg.data.size = 0;
  science_msg.data.data = (float*) malloc(science_msg.data.capacity * sizeof(float));
  
  camera_msg.data.capacity = 1;
  camera_msg.data.size = 0;
  camera_msg.data.data = (uint16_t*) malloc(camera_msg.data.capacity * sizeof(uint16_t));
}

void loop(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  if (millis() - last_cmd_time > cmd_timeout_ms){setDriveNeutralPWM;}
  float []tempAndHumidity = calcTempAndHumidity(analogRead(TEMP_PIN), analogRead(HUMIDITY_PIN));
  temp_msg.data = tempAndHumidity[0];
  humidity_msg.data = tempAndHumidity[0];

  RCSOFTCHECK(rcl_publish(&pub_temp, &temp_msg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_humidity, &humidity_msg, NULL));

  delay(100); // Makes the system loop at 10 Hz
}