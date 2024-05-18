#include <CleanRTOS.h>        // Include CleanRTOS library for multitasking
#include <PID_v1_bc.h>        // Include PID library for PID control
#include <ESP32Encoder.h>     // Include ESP32 encoder library for motor encoder readings
#include <Wire.h>             // Include Wire library for I2C communication
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// Define motor pins for each motor
#define FRW_PWM_PIN       0  // PWM pin for forward right wheel
#define FRW_IN1_PIN       25  // Motor driver input pin 1 for forward right wheel
#define FRW_IN2_PIN       26  // Motor driver input pin 2 for forward right wheel

#define FLW_PWM_PIN       21  // PWM pin for forward left wheel
#define FLW_IN1_PIN       22  // Motor driver input pin 1 for forward left wheel
#define FLW_IN2_PIN       23  // Motor driver input pin 2 for forward left wheel

#define BRW_PWM_PIN       12  // PWM pin for backward right wheel
#define BRW_IN1_PIN       14  // Motor driver input pin 1 for backward right wheel
#define BRW_IN2_PIN       27  // Motor driver input pin 2 for backward right wheel

#define BLW_PWM_PIN       13  // PWM pin for backward left wheel
#define BLW_IN1_PIN       2   // Motor driver input pin 1 for backward left wheel
#define BLW_IN2_PIN       15  // Motor driver input pin 2 for backward left wheel

#define LEFT_IR_1         35
#define LEFT_IR_2         34
#define MIDDLE_IR         39
#define RIGHT_IR_1        36
#define RIGHT_IR_2        5      

// Flags Definition
#define STOP_STATE              1
#define FORWARD_STATE           2
#define BACKWARD_STATE          3
#define RIGHT_STATE             4
#define LEFT_STATE              5
#define CLOCKWISE_STATE         6
#define COUNTER_CLOCKWISE_STATE 7
#define SLIGHT_RIGHT_STATE      8
#define SLIGHT_LEFT_STATE       9

int current_state;
double detection_flag = 0;
int aruco_flag = 0;

// Define task handles for each motor
TaskHandle_t FRW_pidTaskHandle, FLW_pidTaskHandle, BRW_pidTaskHandle, BLW_pidTaskHandle, NavigationHandle;

// Function prototype for callback function
void irFlagCallback(const std_msgs::Float64& msg);
void arucocallback(const std_msgs::Int32& msg);

// ROS node handle
ros::NodeHandle nh;

// ROS subscriber for ir_flag topic
std_msgs::Float64 ir_flag_msg;
ros::Subscriber<std_msgs::Float64> ir_flag_sub("ir_flag", &irFlagCallback);
ros::Subscriber<std_msgs::Int32> aruco_sub("/aruco_marker_flag", &arucocallback);

// Tasks prototypes
//void LineFollower_Task(void *pvParameters);
void FRW_pidTask(void *pvParameters);
void FLW_pidTask(void *pvParameters);
void BRW_pidTask(void *pvParameters);
void BLW_pidTask(void *pvParameters);
void Navigation(void *pvParameters);

// Functions prototypes
void Stop(void);
void moveForward(void);
void moveBackward(void);
void moveRight(void);
void moveLeft(void);
void rotateClockwise(void);
void rotateCounterClockwise(void);
void slightRight(void);
void slightLeft(void);
void enterRoom(void);

void setup() {

//  pinMode(LEFT_IR_1, INPUT);
//  pinMode(LEFT_IR_2, INPUT);
//  pinMode(MIDDLE_IR, INPUT);
//  pinMode(RIGHT_IR_1, INPUT);
//  pinMode(RIGHT_IR_2, INPUT);

  nh.initNode();
  nh.subscribe(ir_flag_sub);
  nh.subscribe(aruco_sub);

  pinMode(FRW_PWM_PIN, OUTPUT);
  pinMode(FRW_IN1_PIN, OUTPUT);
  pinMode(FRW_IN2_PIN, OUTPUT);

  pinMode(FLW_PWM_PIN, OUTPUT);
  pinMode(FLW_IN1_PIN, OUTPUT);
  pinMode(FLW_IN2_PIN, OUTPUT);

  pinMode(BRW_PWM_PIN, OUTPUT);
  pinMode(BRW_IN1_PIN, OUTPUT);
  pinMode(BRW_IN2_PIN, OUTPUT);

  pinMode(BLW_PWM_PIN, OUTPUT);
  pinMode(BLW_IN1_PIN, OUTPUT);
  pinMode(BLW_IN2_PIN, OUTPUT);

  // Create line follower leading task
//  xTaskCreate(
//    LineFollower_Task,        // Task function
//    "LineFollower_Task",      // Task name
//    10000,                    // Stack size
//    NULL,                     // Task parameters
//    1,                        // Priority
//    &LineFollower_TaskHandle  // Task handle
//  );

  // Create tasks for each motor
  xTaskCreate(
    FRW_pidTask,              // Task function
    "FRW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &FRW_pidTaskHandle        // Task handle
  );
  // Create tasks for each motor
  xTaskCreate(
    FLW_pidTask,              // Task function
    "FLW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &FLW_pidTaskHandle        // Task handle
  );
  xTaskCreate(
    BRW_pidTask,              // Task function
    "BRW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &BRW_pidTaskHandle        // Task handle
  );
  xTaskCreate(
    BLW_pidTask,              // Task function
    "BLW_PID_Task",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &BLW_pidTaskHandle        // Task handle
  );
    xTaskCreate(
    Navigation,              // Task function
    "Navigation",           // Task name
    10000,                    // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    &NavigationHandle        // Task handle
  );
}

void loop() {
  // Main loop, tasks are handled by CleanRTOS tasks
}

void irFlagCallback(const std_msgs::Float64& msg) {
  // Update current_state based on received message
  /*if (msg.data == 1) {
    // Prohibit forward motion
    Stop();
  } else if (msg.data == 2) {
    // Prohibit left motion
    // Example: moveRight();
    moveRight();
  } else if (msg.data == 3) {
    // Prohibit right motion
    // Example: moveLeft();
    moveLeft();
  }*/
  detection_flag = msg.data ;
}
void arucocallback(const std_msgs::Int32& msg) {
  aruco_flag = msg.data;
  
}

//void LineFollower_Task(void *pvParameters) {
//  (void) pvParameters;
//
//  for (;;) {
//    if (Serial.available() > 0) {
//      char command = Serial.read();
//      switch (command) {
//        case 'w':
//          moveForward();
//          break;
//        case 's':
//          moveBackward();
//          break;
//        case 'a':
//          moveLeft();
//          break;
//        case 'd':
//          moveRight();
//          break;
//        case 'e':
//          rotateClockwise();
//          break;
//        case 'r':
//          rotateCounterClockwise();
//          break;
//        case 'q':
//          Stop();
//          break;
//        // Add more cases for other commands if needed
//      }
//    }
//    nh.spinOnce();
//    vTaskDelay(5);
//  }
//}

// Task to control FRW motor using PID
void Navigation(void *pvParameters) {
  (void)  pvParameters;
  while(1)
  {
    if(aruco_flag)
    {
      enterRoom();
    }
    else
    {
      //moveForward();
    }
  //moveForward();
  }
  
  
}

void FRW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(FRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 125);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 125);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 225);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 225);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(FRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FRW_IN2_PIN, HIGH);
      analogWrite(FRW_PWM_PIN, 125);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 80);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(FRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FRW_IN2_PIN, LOW);
      analogWrite(FRW_PWM_PIN, 125);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(5);
  }
}

// Task to control FLW motor using PID
void FLW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(FLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 125);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 225);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 225);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(FLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, 80);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(5);
  }
}

// Task to control BRW motor using PID
void BRW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(BRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 125);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 125);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 225);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 225);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, 125);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 80);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(BRW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, 125);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(5);
  }
}

// Task to control FLW motor using PID
void BLW_pidTask(void *pvParameters) {
  (void)  pvParameters;

  for (;;) {
    if (current_state == STOP_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 0);
    }
    else if (current_state == FORWARD_STATE) 
    {
      digitalWrite(BLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 125);
    } 
    else if(current_state == BACKWARD_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 125);
    }
    else if(current_state == RIGHT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 225);
    }
    else if(current_state == LEFT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 225);
    }
    else if(current_state == CLOCKWISE_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 125);
    }
    else if(current_state == COUNTER_CLOCKWISE_STATE)
    {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_RIGHT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 125);
    }
    else if(current_state == SLIGHT_LEFT_STATE)
    {
      digitalWrite(BLW_IN1_PIN, HIGH);       // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, 80);
    }
    nh.spinOnce();
    // Delay for task execution
    vTaskDelay(5);
  }
}

void Stop(void)
{
  current_state = STOP_STATE;
}
void moveForward(void)
{
  current_state = FORWARD_STATE;
}
void moveBackward(void)
{
  current_state = BACKWARD_STATE;
}
void moveRight(void)
{
  current_state = RIGHT_STATE;
}
void moveLeft(void)
{
  current_state = LEFT_STATE;
}
void rotateClockwise(void)
{
  current_state = CLOCKWISE_STATE;
}
void rotateCounterClockwise(void)
{
  current_state = COUNTER_CLOCKWISE_STATE;
}
void slightRight(void)
{
  current_state = SLIGHT_RIGHT_STATE;
}
void slightLeft(void)
{
  current_state = SLIGHT_LEFT_STATE;
}
void enterRoom(void)
{
  Stop();
  delay(1000);
  rotateClockwise();
  delay(1500);
  moveLeft();
  delay(2500);
  moveForward();
  delay(2000);
  Stop();
  aruco_flag = 0;
  delay(10000);
  
  
}
