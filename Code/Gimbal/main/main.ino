#include <DynamixelShield.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
// #include <Adafruit_AHRS.h>

// Initialize the Dynamixel Shield
DynamixelShield dxl;

// Initialize the ICM20948 IMU
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;

// pick your filter! slower == better quality output
// Adafruit_Mahony filter;  // fastest/smallest

#define FILTER_UPDATE_RATE_HZ 25

const uint8_t MOTOR_YAW = 1;    // Yaw motor
const uint8_t MOTOR_PITCH = 3;  // Pitch motor
const uint8_t MOTOR_ROLL = 2;   // Roll motor

const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_PRESENT_POSITION = 132;

const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;

const uint16_t DXL_MINIMUM_POSITION_VALUE = 0;
const uint16_t DXL_MAXIMUM_POSITION_VALUE = 4095;

const double DXL_CENTER_POSITION = 2048.0; // Center position for Dynamixel (assuming 12-bit resolution)

// Variables to store the current position of the motors
double current_pitch_position = 0.0;
double current_roll_position = 0.0;
double current_yaw_position = 0.0;

uint32_t timestamp;

float old_roll = 0.0, old_pitch = 0.0, old_heading = 0.0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(57600);
  while (!Serial);

  // Set Port baudrate
  dxl.begin(57600);

  // Initialize ICM20948
  if (!icm.begin_I2C()) {
    while (1) {
      // Serial.println("Failed to initialize ICM20948");
      delay(10);
    }
  }

  // Serial.println("ICM20948 initialized!");

  icm_temp = icm.getTemperatureSensor();
  icm_accel = icm.getAccelerometerSensor();
  icm_gyro = icm.getGyroSensor();
  icm_mag = icm.getMagnetometerSensor();

  // Set Operating Mode to Extended Position Control Mode
  dxl.setOperatingMode(MOTOR_PITCH, OP_EXTENDED_POSITION);
  dxl.setOperatingMode(MOTOR_ROLL, OP_EXTENDED_POSITION);
  dxl.setOperatingMode(MOTOR_YAW, OP_EXTENDED_POSITION);

  // Enable torque for the motors
  dxl.torqueOn(MOTOR_PITCH);
  dxl.torqueOn(MOTOR_ROLL);
  dxl.torqueOn(MOTOR_YAW);

  // Serial.println("Motors initialized and torque enabled!");

  dxl.setGoalPosition(MOTOR_PITCH, 1024);
  dxl.setGoalPosition(MOTOR_ROLL, 2048);
  dxl.setGoalPosition(MOTOR_YAW, 0);

  // filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}

void loop() {
  // float roll, pitch, heading;
  // float gx, gy, gz;

  // if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
  //   return;
  // }

  // timestamp = millis();

  // sensors_event_t accel;
  // sensors_event_t gyro;
  // sensors_event_t temp;
  // sensors_event_t mag;
  // icm_temp->getEvent(&temp);
  // icm_accel->getEvent(&accel);
  // icm_gyro->getEvent(&gyro);
  // icm_mag->getEvent(&mag);

  // gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  // gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  // gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // // Update the SensorFusion filter
  // filter.update(gx, gy, gz, 
  //               accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
  //               mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // roll = filter.getRoll();
  // pitch = filter.getPitch();
  // heading = filter.getYaw();

  // float roll_change = roll - old_roll;
  // float pitch_change = pitch - old_pitch;
  // float heading_change = heading - old_heading;

  // old_roll = roll;
  // old_pitch = pitch;
  // old_heading = heading;

  // // Serial.print("Change: ");
  // // Serial.print(heading_change);
  // // Serial.print(", ");
  // // Serial.print(pitch_change);
  // // Serial.print(", ");
  // // Serial.println(roll_change);

  // // Update current positions based on changes
  // current_pitch_position += pitch_change;
  // current_roll_position += roll_change;
  // current_yaw_position += heading_change;

  // // Constrain current positions to within -180 to 180 degrees range
  // current_pitch_position = constrain(current_pitch_position, -180.0, 180.0);
  // current_roll_position = constrain(current_roll_position, -180.0, 180.0);
  // current_yaw_position = constrain(current_yaw_position, -180.0, 180.0);

  // // Convert current positions to Dynamixel positions
  // uint16_t pitch_position = DXL_CENTER_POSITION + (current_pitch_position * (4095.0 / 360.0));
  // uint16_t roll_position = DXL_CENTER_POSITION + (current_roll_position * (4095.0 / 360.0));
  // uint16_t yaw_position = DXL_CENTER_POSITION + (current_yaw_position * (4095.0 / 360.0));

  // // Constrain positions to valid range
  // pitch_position = constrain(pitch_position, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
  // roll_position = constrain(roll_position, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
  // yaw_position = constrain(yaw_position, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);

  // // Serial.print("Motor positions - Pitch: ");
  // // Serial.print(pitch_position);
  // // Serial.print(", Roll: ");
  // // Serial.print(roll_position);
  // // Serial.print(", Yaw: ");
  // // Serial.println(yaw_position);

  // // Set motor positions
  // dxl.setGoalPosition(MOTOR_PITCH, pitch_position);
  // dxl.setGoalPosition(MOTOR_ROLL, roll_position);
  // dxl.setGoalPosition(MOTOR_YAW, yaw_position);
}
