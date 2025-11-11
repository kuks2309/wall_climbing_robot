#include "I2CComm_Single.h"
#include "LS7166.h"
#include "PIDController.h"
#include "remote_control.h"
#include <Arduino_FreeRTOS.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <avr/wdt.h> // Watchdog timer library
#include <semphr.h>


// ============================================================================
// Pin Definitions and Hardware Configuration
// ============================================================================

// Motor driver pins (H-Bridge control)
#define MOTOR1_EN1 2 // Motor 1 direction pin 1
#define MOTOR1_EN2 3 // Motor 1 direction pin 2

// Servo control pins
#define SERVO1_PIN 6          // Servo 1 PWM control pin
#define SERVO1_NEUTRAL 93     // Servo 1 neutral position (degrees)
#define SERVO1_ANGLE_LIMIT 35 // Servo 1 angle range: ±35 degrees from neutral

// Encoder SPI chip select pins
#define LS7166_CS1 8 // Encoder 1 (Motor 1) CS pin
#define LS7166_CS2 9 // Encoder 2 (Motor 1) CS pin

// Direction reversal settings
#define ENCODER1_REVERSE false  // true: reverse encoder direction, false: normal
#define ENCODER2_REVERSE true   // true: reverse encoder direction, false: normal
#define MOTOR1_REVERSE false    // true: reverse motor direction, false: normal

// Encoder calibration (adjust these values for your robot)
// 1m 당 pulse 수 - 실험 데이터 기반 (20cm 실제 이동 = 3320 펄스)
#define m_1_pulse 16600

// pulse 당 m - 실험 데이터 기반
#define pulse_1_m 1. / 16600.

// Speed control frequency: 50Hz (20ms period)
#define SPEED_CONTROL_FREQ_HZ 50
#define SPEED_CONTROL_PERIOD_MS (1000 / SPEED_CONTROL_FREQ_HZ)

// RC Control Settings
#define RC_MAX_PWM 150  // Maximum PWM value from RC control (0-255)

// ============================================================================
// PID Controller Gains
// ============================================================================

// Motor Speed Control PID (Delta-based, 50Hz)
#define MOTOR1_SPEED_KP 4.5 // Proportional gain
#define MOTOR1_SPEED_KI 0.0 // Integral gain
#define MOTOR1_SPEED_KD 7.0 // Derivative gain

// Motor Position Control PID
#define MOTOR1_POS_KP 0.08 // Proportional gain (reduced to decrease overshoot)
#define MOTOR1_POS_KI 0.03 // Integral gain
#define MOTOR1_POS_KD 1.0  // Derivative gain (increased for better damping)

// ============================================================================
// Watchdog Configuration
// ============================================================================
#define WATCHDOG_ENABLE true // Enable/disable watchdog timer
#define WATCHDOG_TIMEOUT                                                                                               \
    WDTO_2S // Watchdog timeout: 2 seconds
            // Options: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS,
            //          WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, WDTO_8S

// Control modes
enum ControlMode
{
    MODE_PWM = 0,         // Direct PWM control (NO FEEDBACK)
    MODE_SPEED_CONTROL,   // Speed control with PID
    MODE_POSITION_CONTROL // Position control with PID
};

// Motor control structure
typedef struct
{
    int target_speed;             // internal: raw value or delta pulse
    int current_speed;            // in delta pulse per control cycle
    int16_t target_speed_mms;     // in mm/s (for external interface)
    int32_t target_position;      // in pulse
    int32_t current_position;     // in pulse
    int32_t target_position_mm;   // in mm (for external interface)
    int pwm_output;
    ControlMode mode;
} MotorControl;

// Global objects
Servo servo1;
LS7166 encoder(LS7166_CS1, LS7166_CS2, ENCODER1_REVERSE, ENCODER2_REVERSE);
I2CComm_Single i2c;
RemoteControl* remote_control = nullptr;

// Control structures
MotorControl motor1_ctrl = {0, 0, 0, 0, 0, 0, 0, MODE_PWM};

// Semaphores for data protection
SemaphoreHandle_t xMotor1Semaphore;
SemaphoreHandle_t xSerialSemaphore;

// PID Controllers (using defined gains)
PIDController motor1SpeedPID(MOTOR1_SPEED_KP, MOTOR1_SPEED_KI, MOTOR1_SPEED_KD);
PIDController motor1PositionPID(MOTOR1_POS_KP, MOTOR1_POS_KI, MOTOR1_POS_KD);

// Function prototypes
void TaskMotorControl(void *pvParameters);
void TaskSerialCommunication(void *pvParameters);

void motor1_speed_control(int speed);

// Helper functions
void setMotorControl(MotorControl *motor_ctrl, SemaphoreHandle_t semaphore, PIDController &speedPID,
                     PIDController &posPID, void (*speed_control_func)(int), uint8_t mode, int16_t data_s,
                     int32_t data_l, float pulse_per_m);
void initializeI2C();

// I2C callback functions
void onMotor1Command(uint8_t mode, int16_t data_s, int32_t data_l);
void onServo1Command(int8_t angle);
void onResetEncodersCommand();

// Motor control functions
void motor1_speed_control(int speed)
{
    // Reverse direction if needed
    if (MOTOR1_REVERSE)
    {
        speed = -speed;
    }

    speed = constrain(speed, -255, 255);

    // Apply PWM to motor driver pins
    if (speed > 0)
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, speed);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR1_EN1, abs(speed));
        analogWrite(MOTOR1_EN2, 0);
        
    }
    else
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, 0);
    }
}

// PID controllers are now class instances (motor1SpeedPID, motor1PositionPID)

// Task 1: Unified Motor Control (50Hz)
void TaskMotorControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SPEED_CONTROL_PERIOD_MS); // 50Hz (20ms)

    int32_t last_enc1 = 0;

    for (;;)
    {
        // Read encoder values
        int32_t current_enc1 = encoder.read_encoder1();

        // Calculate actual delta (encoder counts per 20ms)
        int32_t actual_delta1 = current_enc1 - last_enc1;

        // Motor 1 control
        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            motor1_ctrl.current_speed = actual_delta1;
            motor1_ctrl.current_position = current_enc1;

            if (motor1_ctrl.mode == MODE_SPEED_CONTROL)
            {
                // Speed control: Delta-based PID
                // target_speed already converted from mm/s to delta pulse per cycle
                int32_t delta_error1 = motor1_ctrl.target_speed - actual_delta1;

                motor1SpeedPID.setOutputLimits(-150, 150);
                int output = (int)motor1SpeedPID.compute((float)delta_error1);
                motor1_speed_control(output);
            }
            else if (motor1_ctrl.mode == MODE_POSITION_CONTROL)
            {
                // Position control: Position-based PID
                motor1PositionPID.setOutputLimits(-255, 255);
                int output = (int)motor1PositionPID.compute(motor1_ctrl.target_position, motor1_ctrl.current_position);
                motor1_speed_control(output);
            }
            else if (motor1_ctrl.mode == MODE_PWM)
            {
                // PWM mode: Direct control
                motor1_speed_control(motor1_ctrl.target_speed);
            }

            xSemaphoreGive(xMotor1Semaphore);
        }

        // Update previous encoder value
        last_enc1 = current_enc1;

// Reset watchdog timer
#if WATCHDOG_ENABLE
        wdt_reset();
#endif

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task 3: Serial Communication (debug output every 50ms)
void TaskSerialCommunication(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz (50ms)

    for (;;)
    {
        // Update I2C status and build debug string
        String status = "M1: ";

        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            i2c.setMotor1Status(motor1_ctrl.current_position, motor1_ctrl.current_speed, motor1_ctrl.mode);
            status += "Pos=" + String(motor1_ctrl.current_position);
            status += " Spd=" + String(motor1_ctrl.current_speed);
            status += " Mode=" + String(motor1_ctrl.mode);
            status += " PWM=" + String(motor1_ctrl.target_speed);
            xSemaphoreGive(xMotor1Semaphore);
        }

        // Add servo position
        status += " | S1=" + String(servo1.read());

        // Add remote control status
        if (remote_control)
        {
            status += " | RC:";
            status += remote_control->isConnected() ? "OK" : "DISC";
            status += " Mode=" + String(remote_control->getDriveModeString());
            if (remote_control->isConnected())
            {
                status += " CH0=" + String(remote_control->getChannel(0));
                status += " CH1=" + String(remote_control->getChannel(1));
            }
        }

        // Debug output
        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial.println(status);
            xSemaphoreGive(xSerialSemaphore);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
// Disable watchdog during setup
#if WATCHDOG_ENABLE
    wdt_disable();
#endif

    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    // Initialize hardware
    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo1.write(SERVO1_NEUTRAL);

    // Initialize encoder
    encoder.begin();

    // Reset encoders to 0
    encoder.reset_encoders();
    delay(10);

    // SPI Communication Test
    Serial.println("\n=== SPI Communication Test ===");

    // Test CS pins
    Serial.print("CS1 Pin (");
    Serial.print(LS7166_CS1);
    Serial.println(") - Testing...");
    pinMode(LS7166_CS1, OUTPUT);
    digitalWrite(LS7166_CS1, HIGH);
    Serial.print("  CS1 HIGH: ");
    Serial.println(digitalRead(LS7166_CS1));
    digitalWrite(LS7166_CS1, LOW);
    Serial.print("  CS1 LOW: ");
    Serial.println(digitalRead(LS7166_CS1));
    digitalWrite(LS7166_CS1, HIGH);

    // Test SPI pins (Arduino Mega 2560)
    Serial.println("\nSPI Pins (Mega 2560):");
    Serial.println("  MOSI (51), MISO (50), SCK (52)");
    Serial.print("  MISO state: ");
    Serial.println(digitalRead(50));

    // Raw SPI test
    Serial.println("\nRaw SPI Communication Test:");
    digitalWrite(LS7166_CS1, LOW);
    uint8_t response = SPI.transfer(0x00);
    digitalWrite(LS7166_CS1, HIGH);
    Serial.print("  Encoder 1 SPI response to 0x00: 0x");
    Serial.println(response, HEX);

    // Verify encoder initialization
    int32_t test1 = encoder.read_encoder1();
    int32_t test2 = encoder.read_encoder2();
    Serial.print("\nEncoder 1 initial value: ");
    Serial.println(test1);
    Serial.print("Encoder 2 initial value: ");
    Serial.println(test2);

    if (test1 == -1 || test2 == -1)
    {
        Serial.println("\n!!! WARNING: Encoder returns -1 (0xFFFFFFFF)");
        Serial.println("!!! This indicates SPI communication failure");
        Serial.println("!!! Check (Arduino Mega 2560):");
        Serial.println("!!!   1. MOSI (pin 51) connected to LS7166 DIN");
        Serial.println("!!!   2. MISO (pin 50) connected to LS7166 DOUT");
        Serial.println("!!!   3. SCK (pin 52) connected to LS7166 CLK");
        Serial.println("!!!   4. CS1 (pin 8) connected to Encoder1 CS");
        Serial.println("!!!   5. CS2 (pin 9) connected to Encoder2 CS");
        Serial.println("!!!   6. LS7166 power (VCC/GND)");
    }

    Serial.println("\n*** Please rotate the wheel and observe encoder values ***");
    Serial.println("If values don't change, check encoder A/B signal connections");
    Serial.println("==============================\n");

    // Set integral limits for position control to prevent windup
    motor1PositionPID.setIntegralLimit(1000.0);

    // Create semaphores
    xMotor1Semaphore = xSemaphoreCreateMutex();
    xSerialSemaphore = xSemaphoreCreateMutex();

    // Initialize I2C communication
    initializeI2C();

    // Initialize Remote Control (SBUS on Serial2)
    // Serial2 is available on Arduino Mega 2560 (pins 16=RX2, 17=TX2)
    Serial.println("\n=== SBUS Remote Control Initialization ===");
    Serial.println("Serial2: RX2=pin16, TX2=pin17");
    Serial.println("Connect SBUS receiver signal to pin 16 (RX2)");
    remote_control = new RemoteControl(&Serial2);
    remote_control->begin();
    Serial.println("SBUS receiver initialized");
    Serial.println("Drive modes: CH6 controls MANUAL/SEMI_AUTO/FULL_AUTO");
    Serial.println("==========================================\n");

    // Create tasks
    xTaskCreate(TaskMotorControl, "MotorCtrl",
                128, // Stack size
                NULL,
                3, // Priority (highest)
                NULL);

    xTaskCreate(TaskSerialCommunication, "SerialComm",
                256, // Stack size (larger for string handling)
                NULL,
                2, // Priority
                NULL);

    Serial.println("RTOS Motor Control System Started");
    Serial.println("Commands:");
    Serial.println("  I2C Motor Control:");
    Serial.println("    - PWM mode: direct PWM (-255 to 255)");
    Serial.println("    - Speed mode: target speed (mm/s)");
    Serial.println("    - Position mode: target position (mm)");
    Serial.println("  I2C Servo Control:");
    Serial.println("    - Servo angles (degrees, relative to neutral)");
    Serial.println("  SBUS Remote Control:");
    Serial.println("    - MANUAL: RC controls servo and motor");
    Serial.println("    - SEMI_AUTO: I2C controls servo, RC controls motor");
    Serial.println("    - FULL_AUTO: I2C controls both");

// Enable watchdog timer (must be last in setup)
#if WATCHDOG_ENABLE
    wdt_enable(WATCHDOG_TIMEOUT);
    Serial.print("Watchdog enabled with ");
    Serial.print(WATCHDOG_TIMEOUT == WDTO_2S ? "2" : "?");
    Serial.println(" second timeout");
#endif
}

void loop()
{
    // SBUS Remote Control Update (every 100ms = 10Hz)
    static unsigned long last_sbus_update = 0;
    const unsigned long SBUS_UPDATE_INTERVAL = 100; // 100ms

    // SBUS parameters
    const int16_t SBUS_CENTER = 1011;
    const int16_t SBUS_DEADZONE = 20;
    const int16_t SBUS_MIN = 200;
    const int16_t SBUS_MAX = 1800;

    unsigned long current_time = millis();

    if (current_time - last_sbus_update >= SBUS_UPDATE_INTERVAL)
    {
        last_sbus_update = current_time;

        // Update remote control
        if (remote_control)
        {
            remote_control->update();

            // Check connection and drive mode
            if (remote_control->isConnected())
            {
                uint8_t drive_mode = remote_control->getDriveMode();

                // Get channel values
                int16_t ch0 = remote_control->getChannel(0);
                int16_t ch1 = remote_control->getChannel(1);

                // Apply dead zone to CH1 (throttle)
                if (abs(ch1 - SBUS_CENTER) < SBUS_DEADZONE)
                {
                    ch1 = SBUS_CENTER;
                }

                if (drive_mode == RC_MODE_MANUAL)
                {
                    // MANUAL mode: Control both servo and motor via RC
                    // CH0 -> Servo steering
                    int servo_angle = map(ch0, SBUS_MIN, SBUS_MAX, -SERVO1_ANGLE_LIMIT, SERVO1_ANGLE_LIMIT);
                    servo1.write(SERVO1_NEUTRAL + constrain(servo_angle, -SERVO1_ANGLE_LIMIT, SERVO1_ANGLE_LIMIT));

                    // CH1 -> Motor throttle
                    int motor_pwm;
                    if (ch1 < SBUS_CENTER)
                    {
                        // Forward: 200-1011 -> RC_MAX_PWM to 0
                        motor_pwm = map(ch1, SBUS_MIN, SBUS_CENTER, RC_MAX_PWM, 0);
                    }
                    else if (ch1 > SBUS_CENTER)
                    {
                        // Reverse: 1011-1800 -> 0 to -RC_MAX_PWM
                        motor_pwm = map(ch1, SBUS_CENTER, SBUS_MAX, 0, -RC_MAX_PWM);
                    }
                    else
                    {
                        motor_pwm = 0;
                    }

                    // Set motor PWM (MANUAL mode)
                    if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(5)) == pdTRUE)
                    {
                        motor1_ctrl.mode = MODE_PWM;
                        motor1_ctrl.target_speed = motor_pwm;
                        motor1_ctrl.pwm_output = motor_pwm; // Store for debugging
                        xSemaphoreGive(xMotor1Semaphore);
                    }
                }
                else if (drive_mode == RC_MODE_SEMI_AUTO)
                {
                    // SEMI_AUTO mode: Servo via I2C, Motor via RC
                    // CH1 -> Motor throttle only
                    int motor_pwm;
                    if (ch1 < SBUS_CENTER)
                    {
                        motor_pwm = map(ch1, SBUS_MIN, SBUS_CENTER, RC_MAX_PWM, 0);
                    }
                    else if (ch1 > SBUS_CENTER)
                    {
                        motor_pwm = map(ch1, SBUS_CENTER, SBUS_MAX, 0, -RC_MAX_PWM);
                    }
                    else
                    {
                        motor_pwm = 0;
                    }

                    // Set motor PWM (SEMI_AUTO mode)
                    if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(5)) == pdTRUE)
                    {
                        motor1_ctrl.mode = MODE_PWM;
                        motor1_ctrl.target_speed = motor_pwm;
                        motor1_ctrl.pwm_output = motor_pwm; // Store for debugging
                        xSemaphoreGive(xMotor1Semaphore);
                    }
                }
                // FULL_AUTO mode: Both servo and motor controlled by I2C (no RC input)
            }
            else
            {
                // RC disconnected: Stop motor for safety
                if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    motor1_ctrl.mode = MODE_PWM;
                    motor1_ctrl.target_speed = 0;
                    xSemaphoreGive(xMotor1Semaphore);
                }
            }
        }
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

void setMotorControl(MotorControl *motor_ctrl, SemaphoreHandle_t semaphore, PIDController &speedPID,
                     PIDController &posPID, void (*speed_control_func)(int), uint8_t mode, int16_t data_s,
                     int32_t data_l, float pulse_per_m)
{
    if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (mode == I2C_MODE_PWM)
        {
            motor_ctrl->mode = MODE_PWM;
            motor_ctrl->target_speed = data_s;
            speed_control_func(data_s);
        }
        else if (mode == I2C_MODE_SPEED)
        {
            motor_ctrl->mode = MODE_SPEED_CONTROL;
            motor_ctrl->target_speed_mms = data_s;  // Store mm/s value
            // Convert mm/s to delta pulse per 20ms: delta = (mm/s * pulse_per_m) / 1000 / 50Hz
            motor_ctrl->target_speed = (int)((float)data_s * pulse_per_m / 1000.0 / 50.0);
            speedPID.reset();
        }
        else if (mode == I2C_MODE_POSITION)
        {
            motor_ctrl->mode = MODE_POSITION_CONTROL;
            motor_ctrl->target_position_mm = data_l;  // Store mm value
            // Convert mm to pulse: pulse = (mm / 1000) * pulse_per_m
            motor_ctrl->target_position = (int32_t)((float)data_l * pulse_per_m / 1000.0);
            posPID.reset();
        }
        else if (mode == I2C_MODE_POSITION_REL)
        {
            // Relative position mode: add to current position
            motor_ctrl->mode = MODE_POSITION_CONTROL;
            int32_t relative_mm = data_l;
            int32_t relative_pulse = (int32_t)((float)relative_mm * pulse_per_m / 1000.0);
            motor_ctrl->target_position = motor_ctrl->current_position + relative_pulse;
            motor_ctrl->target_position_mm = (int32_t)((float)motor_ctrl->target_position * 1000.0 / pulse_per_m);
            posPID.reset();
        }
        xSemaphoreGive(semaphore);
    }
}

// ============================================================================
// I2C Initialization
// ============================================================================

void initializeI2C()
{
    i2c.begin(I2C_SLAVE_ADDRESS);
    i2c.setMotor1Callback(onMotor1Command);
    i2c.setServo1Callback(onServo1Command);
    i2c.setResetEncoderCallback(onResetEncodersCommand);
    Serial.print("I2C initialized (Single Motor) at address: 0x");
    Serial.println(I2C_SLAVE_ADDRESS, HEX);
}

// ============================================================================
// I2C Callback Functions
// ============================================================================

void onMotor1Command(uint8_t mode, int16_t data_s, int32_t data_l)
{
    // In MANUAL or SEMI_AUTO mode, ignore I2C motor commands (RC controls motor)
    if (remote_control != nullptr && remote_control->isConnected())
    {
        uint8_t drive_mode = remote_control->getDriveMode();
        if (drive_mode == RC_MODE_MANUAL || drive_mode == RC_MODE_SEMI_AUTO)
        {
            // Ignore I2C motor command in MANUAL and SEMI_AUTO modes
            return;
        }
    }

    setMotorControl(&motor1_ctrl, xMotor1Semaphore, motor1SpeedPID, motor1PositionPID, motor1_speed_control, mode,
                    data_s, data_l, m_1_pulse);
}

void onServo1Command(int8_t angle)
{
    // In MANUAL mode, ignore I2C servo commands (RC controls servo)
    if (remote_control != nullptr && remote_control->isConnected())
    {
        uint8_t drive_mode = remote_control->getDriveMode();
        if (drive_mode == RC_MODE_MANUAL)
        {
            // Ignore I2C servo command in MANUAL mode only
            return;
        }
    }

    // Input range: -SERVO1_ANGLE_LIMIT to +SERVO1_ANGLE_LIMIT degrees
    // Output: SERVO1_NEUTRAL ± angle
    int8_t constrained_angle = constrain(angle, -SERVO1_ANGLE_LIMIT, SERVO1_ANGLE_LIMIT);
    int servo_position = SERVO1_NEUTRAL + constrained_angle;
    servo1.write(constrain(servo_position, 0, 180));
}

void onResetEncodersCommand()
{
    encoder.reset_encoders();
}