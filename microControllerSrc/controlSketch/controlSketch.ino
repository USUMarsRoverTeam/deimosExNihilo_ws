#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <PWMServo.h>
#include <PololuMaestro.h>

#define maestroSerial Serial1
MicroMaestro maestro(maestroSerial);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// rcl_subscription_t sub_twist;
// rcl_subscription_t sub_arm;
rcl_subscription_t sub_arm_vel;
rcl_subscription_t sub_arm_home;

rcl_publisher_t pub_arm_pos;

// geometry_msgs__msg__Twist drivetrain_msg;
//std_msgs__msg__UInt16MultiArray arm_msg;
std_msgs__msg__Float32MultiArray arm_vel_msg;
std_msgs__msg__Bool arm_home_msg;

// #define PANO_CAM_SERVO_PIN 5
#define LEFT_DRIVE_PIN 2
#define RIGHT_DRIVE_PIN 3
// #define SYRINGE_STEP_PIN 8
// #define SYRINGE_DIR_PIN 9
// #define AUGER_PIN 10
// #define PLUNGER_PIN 11
// #define CAROUSEL_STEP_PIN 12
// #define CAROUSEL_DIR_PIN 13
// #define TEMP_PIN A0
// #define HUMIDITY_PIN A1

const int stepperDelay = 100;
const float tempSlope = 0.04;
const int tempIntercept = -40;
const float humiditySlope = 0.04;
const float humidityIntercept = 0.0;

double wheelBase = 0.92;
double center2edge = wheelBase / 2.0;
double maxLinearVelocity = 0.5;
double maxSpinVelocity = 2.0;
double maxWheelVelocity = maxLinearVelocity + maxSpinVelocity * center2edge;
int pwmVals[3] = {0,90,180};
float deadzone = 0.05;
unsigned long lastCmdTime = 0;
unsigned long cmdTimeout = 1000;

uint16_t currentArmPos[6] = {6000, 7900, 4300, 6000, 4000, 6000};
float armServoVels[6] = {0,0,0,0,0,0};
const uint16_t armServoBounds[6][2] = {
  {5000,8000},
  {4400,8000},
  {4200,7000},
  {6000,7900},
  {4000,8000},
  {3000,9000}
};

const float armVelDeadzone = 0.01;

// PWMServo leftDrive, rightDrive;//, augerMotor, plungerMotor, panoCamServo;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
  }
}

// void setDrivePWM(double left, double right);
// void setDriveNeutralPWM();

// void drivetrainCallback(const void * msgin){
//   const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
//   lastCmdTime = millis();

//   double linVel = (abs(msg->linear.x) < deadzone) ? 0.0 : msg->linear.x;
//   double spinVel = (abs(msg->angular.z) < deadzone) ? 0.0 : msg->angular.z;

//   linVel = constrain(linVel, -maxLinearVelocity, maxLinearVelocity);
//   spinVel = constrain(spinVel, -maxSpinVelocity, maxSpinVelocity);

//   double leftDriveVel = linVel - spinVel * center2edge;
//   double rightDriveVel = linVel + spinVel * center2edge;

//   setDrivePWM(leftDriveVel, rightDriveVel);
// }

void armCallback(const void * msgin){
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  for (size_t i = 0; i < 6 && i < msg->data.size; i++){
    currentArmPos[i] = msg->data.data[i];
    maestro.setTarget(i, msg->data.data[i]);
  }
}

void armVelCallback(const void * msgin){
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  for (size_t i = 0; i < 6 && i < msg->data.size; i++){
    if (abs(msg->data.data[i]) < armVelDeadzone) {
      armServoVels[i] = 0.0;
    } else {
      armServoVels[i] = msg->data.data[i];
    }
  }
}

void armHomeCallback(const void * msgin){
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data){maestro.goHome();}
}

// void setDrivePWM(double left, double right){
//   left = constrain(left, -maxWheelVelocity, maxWheelVelocity);
//   right = constrain(right, -maxWheelVelocity, maxWheelVelocity);

//   int leftPWM, rightPWM;
  
//   if (abs(left) < 0.01) {
//     leftPWM = pwmVals[1];
//   } else {
//     leftPWM = (int)(pwmVals[1] + (left / maxWheelVelocity) * (pwmVals[2] - pwmVals[1]));
//   }
  
//   if (abs(right) < 0.01) {
//     rightPWM = pwmVals[1];
//   } else {
//     rightPWM = (int)(pwmVals[1] - (right / maxWheelVelocity) * (pwmVals[2] - pwmVals[1]));
//   }
  
//   leftDrive.write(leftPWM);
//   rightDrive.write(rightPWM);
// }

// void setDriveNeutralPWM() {
//   leftDrive.write(pwmVals[1]);
//   rightDrive.write(pwmVals[1]);
// }

void updateArmPos(){
  static unsigned long lastArmUpdate = 0;
  
  unsigned long currentTime = millis();
  
  if (lastArmUpdate > 0) {
    float dt = (currentTime - lastArmUpdate) / 1000.0;
    
    for (int i = 0; i < 6; i++){
      if (abs(armServoVels[i]) > armVelDeadzone) {
        float newPos = currentArmPos[i] + armServoVels[i] * dt;
        currentArmPos[i] = constrain(newPos, armServoBounds[i][0], armServoBounds[i][1]);
        maestro.setTarget(i, currentArmPos[i]);
      }
    }
  }
  
  lastArmUpdate = currentTime;
}

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);

  // 1. Spark Max PWM Range Fix
  // Most Spark Max controllers expect 1000us to 2000us. 
  // PWMServo defaults can vary, so we force the limits here.
  // leftDrive.attach(LEFT_DRIVE_PIN, 1000, 2000);
  // rightDrive.attach(RIGHT_DRIVE_PIN, 1000, 2000);
  
  // // Set to Neutral immediately
  // leftDrive.write(90); 
  // rightDrive.write(90);

  // // 2. Hardware Pins
  // augerMotor.attach(AUGER_PIN);
  // plungerMotor.attach(PLUNGER_PIN);
  // panoCamServo.attach(PANO_CAM_SERVO_PIN);
  // panoCamServo.write(67);
  
  // pinMode(SYRINGE_STEP_PIN, OUTPUT);
  // pinMode(SYRINGE_DIR_PIN, OUTPUT);
  // pinMode(CAROUSEL_STEP_PIN, OUTPUT);
  // pinMode(CAROUSEL_DIR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // 3. Micro-ROS Transport & Support
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // 4. Subscriptions
  // RCCHECK(rclc_subscription_init_default(&sub_twist, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  // RCCHECK(rclc_subscription_init_default(&sub_arm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), "arm_positions"));
  RCCHECK(rclc_subscription_init_default(&sub_arm_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_velocities"));
  RCCHECK(rclc_subscription_init_default(&sub_arm_home, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "arm_home"));

  // 5. CRITICAL: Memory Allocation for MultiArrays
  // Without this, the executor will crash when receiving arm data
  arm_msg.data.capacity = 6;
  arm_msg.data.data = (uint16_t*) malloc(arm_msg.data.capacity * sizeof(uint16_t));
  arm_msg.data.size = 0;

  arm_vel_msg.data.capacity = 6;
  arm_vel_msg.data.data = (float*) malloc(arm_vel_msg.data.capacity * sizeof(float));
  arm_vel_msg.data.size = 0;

  // 6. Executor Initialization
  const int num_handles = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  // RCCHECK(rclc_executor_add_subscription(&executor, &sub_twist, &drivetrain_msg, &drivetrainCallback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm, &arm_msg, &armCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm_vel, &arm_vel_msg, &armVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_arm_home, &arm_home_msg, &armHomeCallback, ON_NEW_DATA));
}

void loop() {
  // Handle micro-ROS callbacks (Timeout 10ms for responsiveness)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Safety: Stop motors if we lose the heartbeat from the ROS master
  // if (millis() - lastCmdTime > cmdTimeout) {
  //   setDriveNeutralPWM();
  // }

  // Handle arm velocity integration
  updateArmPos();

  // Small delay to maintain roughly 100Hz frequency
  delay(10);
}