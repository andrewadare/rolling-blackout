//
// Rolling Blackout vehicle control firmware written for Teensy 3.2
//
#include "Encoder.h"
#include "PIDControl.h"
#include "NAxisMotion.h"
#include <Wire.h>

#define DEBUG 0
#define RADIO_CONTROL HIGH
#define AUTONOMOUS LOW

// Pin definitions
// * A4, A5 occupied for I2C.
// * HW UART for Pololu 24v23 driver (#1383) on Serial1 pins 0,1
#define THROTTLE_RX_PIN        0
#define THROTTLE_TX_PIN        1
#define THROTTLE_ERR_PIN       2
#define THROTTLE_RST_PIN       3
#define CONTROL_MODE_PIN       4 // Switch: R/C or auto - not yet wired (TODO)

#define LIDAR_ENC_PIN          7 // Lidar encoder ch. A (purple)
#define LIDAR_REV_PIN          8 // Photointerrupter (gray)
#define LIDAR_PWM_PIN          9 // Input w/580 ohm pulldown (yellow).
#define ODO_ENC_PIN_A         11
#define ODO_ENC_PIN_B         12
#define LED_PIN               13
#define STEER_ANGLE_PIN       14 // A0
#define IMU_I2C_SDA           18 // A4
#define IMU_I2C_SCL           19 // A5
#define STEER_DIR_PIN         20 // HIGH: right, LOW: left (white)
#define STEER_PWM_PIN         21 // (gray)
#define RC_STEER_INPUT_PIN    22 // From R/C receiver CH1
#define RC_THROTTLE_INPUT_PIN 23 // From R/C receiver CH3

// Global constants
const int PRINT_INTERVAL = 20;          // Time between comm updates (ms)
const int LIDAR_PERIOD = 1346;          // Encoder pulses per rotation
const int ADC_FULL_LEFT  = 430;         // ADC reading at left steer angle limit
const int ADC_FULL_RIGHT = 830;         // and at right limit
const float DEG_FULL_LEFT  = -27.0;     // Angle in degrees at left and right
const float DEG_FULL_RIGHT =  27.0;     // limits
const float METERS_PER_TICK = 1.07/700; // Wheel circumference/(ticks per rev)
const float STEER_PID_DEADBAND = 0.25;  // Suppress low-effort motor "struggle"
unsigned long STEER_PID_INTERVAL = 10;  // ms
unsigned long THROTTLE_PID_INTERVAL = 25;  // ms


// *****************************************************************************
// The set bits of the value read from this variable indicate the errors that
// are currently stopping the motor. The motor can only be driven when this
// register has a value of 0.
// - Bit 0: Safe Start Violation
// - Bit 1: Required Channel Invalid
// - Bit 2: Serial Error
// - Bit 3: Command Timeout
// - Bit 4: Limit/Kill Switch
// - Bit 5: Low VIN
// - Bit 6: High VIN
// - Bit 7: Over Temperature
// - Bit 8: Motor Driver Error
// - Bit 9: ERR Line High
// - Bits 10-15: reserved
#define ERROR_STATUS_ID 0

char* errorStatusMessage[] = {
  "safe start violation",
  "required channel invalid",
  "serial error",
  "command timeout",
  "limit/kill switch",
  "low VIN",
  "high VIN",
  "over temperature",
  "motor driver error",
  "ERR line high"
};

// The set bits of this variable indicate things that are currently limiting the motor controller.
// - Bit 0: Motor is not allowed to run due to an error or safe-start violation.
// - Bit 1: Temperature is active reducing target speed.
// - Bit 2: Max speed limit is actively reducing target speed (target speed > max speed).
// - Bit 3: Starting speed limit is actively reducing target speed to zero (target speed < starting speed).
// - Bit 4: Motor speed is not equal to target speed because of acceleration, deceleration, or brake duration limits.
// - Bit 5: RC1 is configured as a limit/kill switch and the switch is active (scaled value ≥ 1600).
// - Bit 6: RC2 limit/kill switch is active (scaled value ≥ 1600).
// - Bit 7: AN1 limit/kill switch is active (scaled value ≥ 1600).
// - Bit 8: AN2 limit/kill switch is active (scaled value ≥ 1600).
// - Bit 9: USB kill switch is active.
// - Bits 10-15: reserved
#define LIMIT_STATUS_ID 3

char* limitStatusMessage[] = {
  "safe start violation",
  "temperature limit",
  "target > max speed",
  "target < start speed",
  "acc/dec/brake limits",
  "RC1 limit/kill",
  "RC2 limit/kill",
  "AN1 limit/kill",
  "AN2 limit/kill",
  "USB limit/kill"
};

#define TARGET_SPEED_ID   20  // int16 (-3200 to +3200) - Motor target speed
#define MOTOR_SPEED_ID    21  // int16 (-3200 to +3200) - Current motor speed
#define BRAKE_AMOUNT_ID   22  // uint16 (0-32) - 0 = full coast; 32 = full brake; 255 if speed=0.
#define INPUT_VOLTAGE_ID  23  // uint16 - Voltage on VIN in mV
#define TEMPERATURE_ID    24  // uint16 - Board temp. in units of 0.1 deg C
#define RC_PERIOD_ID      26  // uint16 - If there is a valid signal on RC1, this variable contains the signal period. Otherwise, this variable has a value of 0. (0.1 ms)
#define BAUD_RATE_ID      27  // uint16 - Value of the controller’s baud rate register (BRR). Convert to units of bps with the equation 72,000,000/BRR. In automatic baud detection mode, BRR has a value of 0 until the controller has detected the baud rate. seconds per 7.2e7 bits
#define SYS_TIME_LOW_ID   28  // uint16 - Two lower bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up. ms
#define SYS_TIME_HIGH_ID  29  // uint16 - Two upper bytes of the number of milliseconds that have elapsed since the controller was last reset or powered up. 65,536 ms

// Variable IDs for motor limits. If IDs 0-3 are used, the limit is
// applied to both fwd and rev directions. If IDs > 3 are used, the limit is
// applied only to the specified direction.
#define SPEED_LIMIT_ID 0 // valid limitValue: 0-3200 for 0-100%
#define ACCEL_LIMIT_ID 1 // valid limitValue: 0-3200 (0 = no limit)
#define DECEL_LIMIT_ID 2 // valid limitValue: 0-3200 (0 = no limit)

// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  Serial1.write(0x83);
}

// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  if (speed < 0)
  {
    Serial1.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    Serial1.write(0x85);  // motor forward command
  }
  Serial1.write(speed & 0x1F);
  Serial1.write(speed >> 5);
}

uint8_t setMotorLimit(unsigned char limitID, uint16_t limitValue)
{
  Serial1.write(0xA2);
  Serial1.write(limitID);
  Serial1.write(limitValue & 0x7F);
  Serial1.write(limitValue >> 7);
  return Serial1.read();
}

// returns the specified variable as an unsigned integer.
// if the requested variable is signed, the value returned by this function
// should be typecast as an int.
int getValue(unsigned char variableID)
{
  Serial1.write(0xA1);
  Serial1.write(variableID);

  // These might return -1 in case of a timeout
  int lowByte = Serial1.read();
  int highByte = Serial1.read();
  if (lowByte == -1 || highByte == -1)
    return -1;
  return lowByte + 256*highByte;
}
// *****************************************************************************


// PID parameters
// Notes: 1, 0.001, 0 causes large slow oscillations that damp out over ~5sec
// Notes: 1, 0.0001, 0 drifts slowly to target
// Notes: 1, 0.0005, 100 - still big oscillations
float kp = 2, ki = 0.0002, kd = 100;

// Center steering position in "PID units" (i.e. scaled to the [-1,1] interval)
float initialSteerSetpoint = 0;

float adcToDegrees(int adc)
{
  return (float)map(adc, ADC_FULL_LEFT, ADC_FULL_RIGHT, 100*DEG_FULL_LEFT, 100*DEG_FULL_RIGHT)/100;
}

// Convert steering readout in ADC units to PID units in [-1,1]
float adcToPidUnits(int adc)
{
  return (float)map(adc, ADC_FULL_LEFT, ADC_FULL_RIGHT, -1000, 1000)/1000;
}

// Convert steer angle in degrees to PID units in [-1,1]
float degreesToPidUnits(float angle)
{
  return (float)map(1000*angle, 1000*DEG_FULL_LEFT, 1000*DEG_FULL_RIGHT, -1000, 1000)/1000;
}

Encoder odometer(ODO_ENC_PIN_A, ODO_ENC_PIN_B);

PIDControl steerPid(kp, ki, kd, initialSteerSetpoint, STEER_PID_INTERVAL);
float throttleSetpoint = 0;

// Bosch BNO055 absolute orientation sensor
NAxisMotion imu;

// Timer for update intervals in ms
elapsedMillis printTimer = 0;
elapsedMillis steerPidTimer = 0;
elapsedMillis throttlePidTimer = 0;

// Timer for Lidar signal (PWM pulse width measurement)
elapsedMicros lidarTimer = 0;

// Timers for PWM pulse widths from R/C receiver (1000-2000 us @ 50Hz)
elapsedMicros rcSteerPwmTimer = 0;
elapsedMicros rcThrottlePwmTimer = 0;

volatile unsigned int rcSteerPulseWidth = 0; // microseconds
volatile unsigned int rcThrottlePulseWidth = 0; // microseconds

volatile unsigned int lidarPulseWidth = 0; // microseconds
volatile unsigned int lidarAngleCounter = 0; // Single-channel pulse counter

// These are copied from their volatile counterparts to avoid asynchronous mutation
unsigned int lidarRange = 0; // cm
unsigned int lidarAngle = 0; // deg
unsigned int rcSteerValue = 0; // same as rcSteerPulseWidth
unsigned int rcThrottleValue = 0; // same as rcThrottlePulseWidth
long odometerValue = 0;

// Distance traveled by vehicle in meters
float odometerDistance = 0;

// Steering angle and limits. Sense is positive clockwise to match IMU heading.
unsigned int adc16 = 0;  // 16*ADC value from turnpot
float steeringAngle = 0;  // Front wheel angle (deg)

String cmd = "";

// From the PID output value in [-1,+1], set the direction
// and the 8-bit PWM duty cycle for the steering motor.
void updateSteering(float pidOutput)
{
  // Use |pidOutput| for linear mapping. Sign used below for STEER_DIR_PIN.
  if (fabs(pidOutput) < STEER_PID_DEADBAND)
    pidOutput = 0;

  int steerMotorPwmVal = map(1e6*fabs(pidOutput), 0, 1e6, 0, 255);
  steerMotorPwmVal = constrain(steerMotorPwmVal, 0, 255);

#if DEBUG
  Serial.print(degreesToPidUnits(steeringAngle));
  if (pidOutput > 0)
    Serial.print(" Set +");
  else
    Serial.print(" Set -");
  Serial.println(steerMotorPwmVal);
#endif

  digitalWrite(STEER_DIR_PIN, pidOutput > 0 ? HIGH : LOW);
  analogWrite(STEER_PWM_PIN, steerMotorPwmVal);
}

void setup()
{
  pinMode(CONTROL_MODE_PIN, INPUT_PULLUP);
  pinMode(RC_STEER_INPUT_PIN, INPUT);
  pinMode(RC_THROTTLE_INPUT_PIN, INPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);
  pinMode(STEER_PWM_PIN, OUTPUT);
  pinMode(LIDAR_ENC_PIN, INPUT);
  pinMode(LIDAR_REV_PIN, INPUT);
  pinMode(LIDAR_PWM_PIN, INPUT);
  pinMode(ODO_ENC_PIN_A, INPUT);
  pinMode(ODO_ENC_PIN_B, INPUT);
  pinMode(LED_PIN, OUTPUT);

  steerPid.minOutput = -1; // Full speed left
  steerPid.maxOutput = +1; // Full speed right

  // I2C and IMU sensor initialization
  I2C.begin();
  imu.initSensor();
  imu.setOperationMode(OPERATION_MODE_NDOF);
  imu.setUpdateMode(MANUAL);

  // Attach ISR for pulse width measurement
  attachInterrupt(digitalPinToInterrupt(LIDAR_PWM_PIN), onLidarPinChange, CHANGE);

  // Lidar angle measurement ISRs
  attachInterrupt(digitalPinToInterrupt(LIDAR_ENC_PIN), onLidarEncoderRise, RISING);
  attachInterrupt(digitalPinToInterrupt(LIDAR_REV_PIN), onNewLidarRevolution, RISING);

  // R/C receiver PWM decoders
  attachInterrupt(digitalPinToInterrupt(RC_STEER_INPUT_PIN), onRcSteerPinChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_INPUT_PIN), onRcThrottlePinChange, CHANGE);

  Serial.begin(115200);


  // *****************************************************************************
  Serial1.begin(38400); // UART to motor driver on pins 0,1

  // Briefly reset SMC when MCU starts up (optional)
  pinMode(THROTTLE_RST_PIN, OUTPUT);
  digitalWrite(THROTTLE_RST_PIN, LOW);  // reset SMC
  delay(1);
  pinMode(THROTTLE_RST_PIN, INPUT);  // let SMC run again
  pinMode(THROTTLE_ERR_PIN, INPUT);

  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);

  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  Serial1.write(0xAA);  // send baud-indicator byte
  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run
  exitSafeStart();  // clear the safe-start violation and let the motor run

  // Set all limits to maximum
  setMotorLimit(SPEED_LIMIT_ID, 3200);
  setMotorLimit(ACCEL_LIMIT_ID, 3200);
  setMotorLimit(DECEL_LIMIT_ID, 3200);

  Serial.print("Speed limit: ");
  Serial.println(getValue(SPEED_LIMIT_ID));
  Serial.print("Accel limit: ");
  Serial.println(getValue(ACCEL_LIMIT_ID));
  Serial.print("Decel limit: ");
  Serial.println(getValue(DECEL_LIMIT_ID));

  Serial.print("VIN = ");
  Serial.print(getValue(INPUT_VOLTAGE_ID));
  Serial.println(" mV");
  delay(3000);
  // *****************************************************************************

  // Wait for serial port to connect (for dev work)
  while (!Serial) {;}
}

// Exponentially weighted moving average for integer data. Used for over-
// sampling noisy measurements in a time series.
// When using the result, it must be divided by 16: x16 >> 4.
void ewma(unsigned int x, unsigned int &x16)
{
  // Compute weights like 1/16*(current x) + 15/16*(prev running avg x), except
  // multiplied through by 16 to avoid precision loss from int division:
  // 16*xavg = x + 16*xavg - (16*xavg - 16/2)/16
  x16 = x + x16 - ((x16 - 8) >> 4);
}

// Print calibration status codes (each 0-3) as a 4-digit number
void printCalibStatus()
{
  imu.updateCalibStatus();
  Serial.print(",AMGS:");
  Serial.print(imu.readAccelCalibStatus());
  Serial.print(imu.readMagCalibStatus());
  Serial.print(imu.readGyroCalibStatus());
  Serial.print(imu.readSystemCalibStatus());
}

// Quaternion components (unitless integers - see BNO055 datasheet table 3-31)
void printQuaternions()
{
  imu.updateQuat();
  Serial.print(",qw:");
  Serial.print(imu.readQuatW());
  Serial.print(",qx:");
  Serial.print(imu.readQuatX());
  Serial.print(",qy:");
  Serial.print(imu.readQuatY());
  Serial.print(",qz:");
  Serial.print(imu.readQuatZ());
}

void printLidar()
{
  // Copy volatile data over to stable variables. Use only the latter for calculations.
  noInterrupts();
  lidarAngle = lidarAngleCounter;
  lidarRange = lidarPulseWidth;
  interrupts();

  lidarRange /= 10; // Convert to cm, which is the minimum resolution
  lidarAngle *= 360.0/LIDAR_PERIOD; // Convert to degrees

  if (lidarAngle > 360)
  {
    lidarAngle -= 360;
  }

  Serial.print(",r:");
  Serial.print(lidarRange);
  Serial.print(",b:");
  Serial.print(lidarAngle);
}

void loop()
{
  // *****************************************************************************
  // setMotorSpeed(3200);  // full-speed forward
  // Serial.println((int16_t)getValue(TARGET_SPEED_ID)); // cast signed vals
  // delay(1000);
  // setMotorSpeed(-3200);  // full-speed reverse
  // Serial.println((int16_t)getValue(TARGET_SPEED_ID));
  // delay(1000);

  // write input voltage (in millivolts) to the serial monitor
  // Serial.print("VIN = ");
  // Serial.println(getValue(INPUT_VOLTAGE_ID)); // mV

  // if an error is stopping the motor, write the error status variable
  // and try to re-enable the motor
  if (digitalRead(THROTTLE_ERR_PIN) == HIGH)
  {
    uint16_t errStatus = (uint16_t)getValue(ERROR_STATUS_ID);
    uint16_t limStatus = (uint16_t)getValue(LIMIT_STATUS_ID);
    Serial.print("Error: 0x");
    Serial.println(errStatus, HEX);
    Serial.print("Limit: 0x");
    Serial.println(limStatus, HEX);
    for (uint16_t i=0; i<10; ++i)
    {
      if (errStatus & i)
        Serial.println(errorStatusMessage[i]);
      if (limStatus & i)
        Serial.println(limitStatusMessage[i]);
    }
    Serial.print("VIN = ");
    Serial.println(getValue(INPUT_VOLTAGE_ID)); // mV

    // once all other errors have been fixed,
    // this lets the motors run again
    exitSafeStart();
  }
  // *****************************************************************************

  // Steering control
  if (steerPidTimer >= STEER_PID_INTERVAL)
  {
    steerPidTimer -= STEER_PID_INTERVAL;

    // Copy volatile data over to stable variables. Use only the latter for calculations.
    noInterrupts();
    rcSteerValue = rcSteerPulseWidth;
    interrupts();

    if (digitalRead(CONTROL_MODE_PIN) == RADIO_CONTROL)
    {
      steerPid.setpoint = (float(rcSteerValue) - 1500.)/500;
    }
    // else: from autonomously computed signal (WIP)

    steerPid.update(adcToPidUnits(adc16 >> 4), millis());

    // Temp - dev
    // Serial.print(steerPid.setpoint);
    // Serial.print(",");
    // Serial.println(adcToPidUnits(adc16 >> 4));

    updateSteering(steerPid.output);
  }

  // Throttle control
  if (throttlePidTimer >= THROTTLE_PID_INTERVAL)
  {
    throttlePidTimer -= THROTTLE_PID_INTERVAL;

    noInterrupts();
    rcThrottleValue = rcThrottlePulseWidth;
    interrupts();

    if (digitalRead(CONTROL_MODE_PIN) == RADIO_CONTROL)
    {
      throttleSetpoint = (float(rcThrottleValue) - 1500.)/500;
    }
    // else: from autonomously computed signal (WIP)

    // Deadband
    if (fabs(throttleSetpoint) < 0.1) throttleSetpoint = 0;

    setMotorSpeed((int)(3200*throttleSetpoint));
  }

  // Send out vehicle state every PRINT_INTERVAL milliseconds
  // "t:%d,AMGS:%d%d%d%d,qw:%d,qx:%d,qy:%d,qz:%d,sa:%d,odo:%d,r:%d,b:%d\r\n"
  if (printTimer >= PRINT_INTERVAL)
  {
    printTimer -= PRINT_INTERVAL;

    odometerDistance = METERS_PER_TICK*odometer.read();

    // Serial.print("t:");
    // Serial.print(millis());
    // printCalibStatus();
    // printQuaternions();
    // Serial.print(",sa:");
    // Serial.print(steeringAngle);
    // Serial.print(",odo:");
    // Serial.print(odometerDistance);
    // printLidar();
    // Serial.println();
  }

  // Read potentiometer and update steering angle
  ewma(analogRead(STEER_ANGLE_PIN), adc16);

  steeringAngle = adcToDegrees(adc16 >> 4);
}

// ISR to measure PWM pulse width from LidarLite
void onLidarPinChange()
{
  if (digitalRead(LIDAR_PWM_PIN) == HIGH)
    lidarTimer = 0;
  else
    lidarPulseWidth = lidarTimer;
}

// Measure PWM pulse width from R/C receiver CH1 output
void onRcSteerPinChange()
{
  if (digitalRead(RC_STEER_INPUT_PIN) == HIGH)
    rcSteerPwmTimer = 0;
  else
    rcSteerPulseWidth = rcSteerPwmTimer;
}

// Measure PWM pulse width from R/C receiver CH3 output
void onRcThrottlePinChange()
{
  if (digitalRead(RC_THROTTLE_INPUT_PIN) == HIGH)
    rcThrottlePwmTimer = 0;
  else
    rcThrottlePulseWidth = rcThrottlePwmTimer;
}

// ISRs for lidar encoder and photointerrupter
void onNewLidarRevolution()
{
  lidarAngleCounter = 0;
}
void onLidarEncoderRise()
{
  lidarAngleCounter++;
}

