//
// Rolling Blackout vehicle control firmware written for Teensy 3.2
//
#include "Encoder.h"
#include "PIDControl.h"
#include "NAxisMotion.h"
#include <Wire.h>

#define DEBUG 0

// Pin definitions
// * A4, A5 occupied for I2C.
// * HW UART for Pololu 24v23 driver (#1383) on Serial1 pins 0,1
#define THROTTLE_RX_PIN        0
#define THROTTLE_TX_PIN        1
#define CONTROL_MODE_PIN       2 // Switch: R/C or auto - not yet wired (TODO)
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

#define RADIO_CONTROL HIGH
#define AUTONOMOUS LOW

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

// Bosch BNO055 absolute orientation sensor
NAxisMotion imu;

// Timer for update intervals in ms
elapsedMillis printTimer = 0;
elapsedMillis steerPidTimer = 0;

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

void handleByte(byte b)
{
  if (b == '-')
    cmd = "-";

  // kp
  else if (b == 'p') // increase kp
    kp += 0.01;
  else if (b == 'l') // decrease kp
    kp -= 0.01;

  // ki
  else if (b == 'i') // increase ki
    ki += 0.1;
  else if (b == 'k') // decrease ki
    ki -= 0.1;

  // kd
  else if (b == 'd') // increase kd
    kd += 0.001;
  else if (b == 'c') // decrease kd
    kd -= 0.001;

  else if (b == '.')
    cmd += b;
  else if (isDigit(b))
  {
    byte digit = b - 48;
    cmd += digit;
  }
  else if (b == '\r' || b == '\n')
  {
    // *** For prototyping ***
    // Assume here that cmd is a signed angle setpoint in degrees.
    // Convert to a value in [-1,1] for PID input
    float setPoint = degreesToPidUnits(cmd.toFloat());
    steerPid.setpoint = constrain(setPoint, steerPid.minOutput, steerPid.maxOutput);
    cmd = "";
  }
  else
  {
    steerPid.setPID(kp, ki, kd);
  }
}

void loop()
{

  if (steerPidTimer >= STEER_PID_INTERVAL)
  {
    steerPidTimer -= STEER_PID_INTERVAL;

    // Copy volatile data over to stable variables. Use only the latter for calculations.
    noInterrupts();
    rcSteerValue = rcSteerPulseWidth;
    rcThrottleValue = rcThrottlePulseWidth; // NOTE: may eventually move to throttle PID loop
    interrupts();

    if (digitalRead(CONTROL_MODE_PIN) == RADIO_CONTROL)
    {
      steerPid.setpoint = (float(rcSteerValue) - 1500.)/500;
    }

    steerPid.update(adcToPidUnits(adc16 >> 4), millis());

    // Temp - dev
    Serial.print(steerPid.setpoint);
    Serial.print(",");
    Serial.println(adcToPidUnits(adc16 >> 4));

    updateSteering(steerPid.output);
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

  if (Serial.available() > 0)
  {
    byte rxByte = Serial.read();
    handleByte(rxByte);
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

