#include <CleanRTOS.h>        // Include CleanRTOS library for multitasking
#include <PID_v1_bc.h>        // Include PID library for PID control
#include <ESP32Encoder.h>     // Include ESP32 encoder library for motor encoder readings
#include <Wire.h>             // Include Wire library for I2C communication

// Define motor pins for each motor
#define FRW_PWM_PIN 5         // PWM pin for forward right wheel
#define FRW_IN1_PIN 4         // Motor driver input pin 1 for forward right wheel
#define FRW_IN2_PIN 0         // Motor driver input pin 2 for forward right wheel
#define FRW_ENCODER_PIN_A 18  // Encoder pin A for forward right wheel
#define FRW_ENCODER_PIN_B 19  // Encoder pin B for forward right wheel

#define FLW_PWM_PIN 25        // PWM pin for forward left wheel
#define FLW_IN1_PIN 26        // Motor driver input pin 1 for forward left wheel
#define FLW_IN2_PIN 27        // Motor driver input pin 2 for forward left wheel
#define FLW_ENCODER_PIN_A 17  // Encoder pin A for forward left wheel
#define FLW_ENCODER_PIN_B 16  // Encoder pin B for forward left wheel

#define BRW_PWM_PIN 23        // PWM pin for backward right wheel
#define BRW_IN1_PIN 21        // Motor driver input pin 1 for backward right wheel
#define BRW_IN2_PIN 22        // Motor driver input pin 2 for backward right wheel
#define BRW_ENCODER_PIN_A 32  // Encoder pin A for backward right wheel
#define BRW_ENCODER_PIN_B 33  // Encoder pin B for backward right wheel

#define BLW_PWM_PIN 13        // PWM pin for backward left wheel
#define BLW_IN1_PIN 12        // Motor driver input pin 1 for backward left wheel
#define BLW_IN2_PIN 14        // Motor driver input pin 2 for backward left wheel
#define BLW_ENCODER_PIN_A 34  // Encoder pin A for backward left wheel
#define BLW_ENCODER_PIN_B 35  // Encoder pin B for backward left wheel

// Define PID parameters for each motor
double FLW_setpoint = 8, BRW_setpoint = 8, BLW_setpoint = 8;          // Setpoints for each wheel
double FLW_pulses, BRW_pulses, BLW_pulses;                            // Encoder pulse count variables
double FLW_pulses_prev = 0, BRW_pulses_prev = 0, BLW_pulses_prev = 0; // Previous encoder pulse count variables
double FLW_dt, BRW_dt, BLW_dt;                                        // Time difference variables
double FLW_velocity = 0, BRW_velocity = 0, BLW_velocity = 0;          // Velocity variables
double FLW_output, BRW_output, BLW_output;                            // PID output variables
// PID constants for each wheel
double FLW_Kp = 10, FLW_Ki = 7.5, FLW_Kd = 0.09;
double BRW_Kp = 10, BRW_Ki = 7.5, BRW_Kd = 0.09;
double BLW_Kp = 10, BLW_Ki = 7.5, BLW_Kd = 0.09;
// PID objects for each wheel
PID FLW_pid(&FLW_velocity, &FLW_output, &FLW_setpoint, FLW_Kp, FLW_Ki, FLW_Kd, DIRECT);
PID BRW_pid(&BRW_velocity, &BRW_output, &BRW_setpoint, BRW_Kp, BRW_Ki, BRW_Kd, DIRECT);
PID BLW_pid(&BLW_velocity, &BLW_output, &BLW_setpoint, BLW_Kp, BLW_Ki, BLW_Kd, DIRECT);

// Define encoder objects for each motor
ESP32Encoder FLW_encoder, BRW_encoder, BLW_encoder;

// Define task handles for each motor
TaskHandle_t FLW_pidTaskHandle, BRW_pidTaskHandle, BLW_pidTaskHandle, print_handle;

// Function prototypes
void FLW_pidTask(void *pvParameters);
void BRW_pidTask(void *pvParameters);
void BLW_pidTask(void *pvParameters);
void tasksLogger(void *pvParameters);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Setup motor pins for each motor
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

  // Initialize encoders for each motor
  FLW_encoder.attachFullQuad(FLW_ENCODER_PIN_A, FLW_ENCODER_PIN_B);
  BRW_encoder.attachFullQuad(BRW_ENCODER_PIN_A, BRW_ENCODER_PIN_B);
  BLW_encoder.attachFullQuad(BLW_ENCODER_PIN_A, BLW_ENCODER_PIN_B);

  // Set PID parameters for each motor
  FLW_pid.SetMode(AUTOMATIC);
  FLW_pid.SetSampleTime(10); // Update time in ms
  FLW_pid.SetOutputLimits(-200, 200);
  BRW_pid.SetMode(AUTOMATIC);
  BRW_pid.SetSampleTime(10); // Update time in ms
  BRW_pid.SetOutputLimits(-200, 200);
  BLW_pid.SetMode(AUTOMATIC);
  BLW_pid.SetSampleTime(10); // Update time in ms
  BLW_pid.SetOutputLimits(-200, 200);

  // Create tasks for each motor
  xTaskCreate(
    FLW_pidTask,            // Task function
    "FLW_PID_Task",         // Task name
    10000,                  // Stack size
    NULL,                   // Task parameters
    1,                      // Priority
    &FLW_pidTaskHandle      // Task handle
  );
  xTaskCreate(
    BRW_pidTask,            // Task function
    "BRW_PID_Task",         // Task name
    10000,                  // Stack size
    NULL,                   // Task parameters
    1,                      // Priority
    &BRW_pidTaskHandle      // Task handle
  );
  xTaskCreate(
    BLW_pidTask,            // Task function
    "BLW_PID_Task",         // Task name
    10000,                  // Stack size
    NULL,                   // Task parameters
    1,                      // Priority
    &BLW_pidTaskHandle      // Task handle
  );
  xTaskCreate(
    tasksLogger,            // Task function
    "LoggerTask",           // Task name
    10000,                  // Stack size
    NULL,                   // Task parameters
    1,                      // Priority
    &print_handle           // Task handle    
  );
}

void loop() {
  // Main loop, tasks are handled by CleanRTOS tasks
}

// Task to control FLW motor using PID
void FLW_pidTask(void *pvParameters) {
  (void) pvParameters;

  double cutoff_freq = 5.0;
  double alpha = 0.1;
  double filtered_FLW_output = 0;
  double FLW_prevTime = 0;

  for (;;) {
    // Read encoder value for FLW
    FLW_pulses = FLW_encoder.getCount();
    double FLW_currentTime = millis();
    FLW_dt = FLW_currentTime - FLW_prevTime;
    FLW_velocity = ((FLW_pulses - FLW_pulses_prev) * 1610 / 360) / FLW_dt;

    // Compute PID output for FLW
    FLW_pid.Compute();

    filtered_FLW_output = alpha * FLW_output + (1 - alpha) * filtered_FLW_output;

    // Apply PID output to FLW
    if (filtered_FLW_output > 0) {
      digitalWrite(FLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(FLW_IN2_PIN, LOW);
      analogWrite(FLW_PWM_PIN, filtered_FLW_output);
    } else {
      digitalWrite(FLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(FLW_IN2_PIN, HIGH);
      analogWrite(FLW_PWM_PIN, -filtered_FLW_output);
    }

    // Update variables
    FLW_pulses_prev = FLW_pulses;
    FLW_prevTime = FLW_currentTime;

    // Delay for task execution
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Task to control BRW motor using PID
void BRW_pidTask(void *pvParameters) {
  (void) pvParameters;

  double cutoff_freq = 5.0;
  double alpha = 0.1;
  double filtered_BRW_output = 0;
  double BRW_prevTime = 0;

  for (;;) {
    // Read encoder value for BRW
    BRW_pulses = BRW_encoder.getCount();
    double BRW_currentTime = millis();
    BRW_dt = BRW_currentTime - BRW_prevTime;
    BRW_velocity = ((BRW_pulses - BRW_pulses_prev) * 1610 / 360) / BRW_dt;

    // Compute PID output for BRW
    BRW_pid.Compute();

    filtered_BRW_output = alpha * BRW_output + (1 - alpha) * filtered_BRW_output;

    // Apply PID output to BRW
    if (filtered_BRW_output > 0) {
      digitalWrite(BRW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BRW_IN2_PIN, LOW);
      analogWrite(BRW_PWM_PIN, filtered_BRW_output);
    } else {
      digitalWrite(BRW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BRW_IN2_PIN, HIGH);
      analogWrite(BRW_PWM_PIN, -filtered_BRW_output);
    }

    // Update variables
    BRW_pulses_prev = BRW_pulses;
    BRW_prevTime = BRW_currentTime;

    // Delay for task execution
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Task to control BLW motor using PID
void BLW_pidTask(void *pvParameters) {
  (void) pvParameters;

  double cutoff_freq = 5.0;
  double alpha = 0.1;
  double filtered_BLW_output = 0;
  double BLW_prevTime = 0;

  for (;;) {
    // Read encoder value for BLW
    BLW_pulses = BLW_encoder.getCount();
    double BLW_currentTime = millis();
    BLW_dt = BLW_currentTime - BLW_prevTime;
    BLW_velocity = ((BLW_pulses - BLW_pulses_prev) * 1610 / 360) / BLW_dt;

    // Compute PID output for BLW
    BLW_pid.Compute();

    filtered_BLW_output = alpha * BLW_output + (1 - alpha) * filtered_BLW_output;

    // Apply PID output to BLW
    if (filtered_BLW_output > 0) {
      digitalWrite(BLW_IN1_PIN, HIGH);      // Forward direction
      digitalWrite(BLW_IN2_PIN, LOW);
      analogWrite(BLW_PWM_PIN, filtered_BLW_output);
    } else {
      digitalWrite(BLW_IN1_PIN, LOW);       // Reverse direction
      digitalWrite(BLW_IN2_PIN, HIGH);
      analogWrite(BLW_PWM_PIN, -filtered_BLW_output);
    }

    // Update variables
    BLW_pulses_prev = BLW_pulses;
    BLW_prevTime = BLW_currentTime;

    // Delay for task execution
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Task to log motor velocities
void tasksLogger(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Print velocities to serial monitor
    Serial.print(FLW_velocity);
    Serial.print(",");
    Serial.print(BRW_velocity);
    Serial.print(",");
    Serial.println(BLW_velocity);

    // Delay for task execution
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
