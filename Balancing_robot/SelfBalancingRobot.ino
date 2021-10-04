#include <Wire.h>

const int mpuAddress = 0x68;

int16_t accX_raw, accZ_raw, gyroY_raw;      // raw values as measured by the MPU6050 sensor
int gyroOffsetY = -200; // offset values for accelerometer and gyroscope
double acc_angle, gyro_angle, error;        // angles measured by the acceleromter and gyroscpoe
unsigned long loopTime;                     // loop time determines the time to be taken by each loop and elapsed time

// PID values
float pid_Kp;
float pid_Ki;
float pid_Kd;

int start = 0;                                // the robot starts balancing when start variable is set to 1

void MPU_Initialize(void) // initializing the MPU6050
{
  // waking up the MPU6050 form low power consumption mode
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x6B);
  Wire.write(0x04);
  Wire.endTransmission(true);

  // configuring the full scale value of the accelerometer to +/- 4g
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission(true);

  // configuring the full scale value of the gyroscope to +/- 250dps
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //configuring the digital low pass filter to 43Hz
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  return;
}

void acc_raw_calc(void) // function to obtain the raw accelerometer values from the MPU6050
{
  // reading raw accelration data along X axis
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(mpuAddress, 2);
  accX_raw = Wire.read() << 8 | Wire.read();
  //accX_raw -= 400;

  // reading raw acceleration data along Z axis
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3F);
  Wire.endTransmission(true);
  Wire.requestFrom(mpuAddress, 2);
  accZ_raw = Wire.read() << 8 | Wire.read();

  return;
}


void gyro_raw_calc(void) // function to obtain the raw gyroscope values from the MPU6050
{
  // reading raw gyroscope data along Y axis
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x45);
  Wire.endTransmission(true);
  Wire.requestFrom(mpuAddress, 2);
  gyroY_raw = Wire.read() << 8 | Wire.read();
  gyroY_raw -= gyroOffsetY;

  return;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  MPU_Initialize();

  Serial.print("gyro Y offset : ");
  Serial.println(gyroOffsetY);

  loopTime = micros() + 4000;
}

void loop() {
  // calculate the acclerometer and gyroscope raw values
  acc_raw_calc();
  gyro_raw_calc();

  // claculate the angle as measured by the acclerometer
  acc_angle = -1 * atan((float)accX_raw / accZ_raw) * 57.29;
  Serial.print("Acc angle : ");
  Serial.println(acc_angle);

  if (start == 0 && acc_angle > -1.5 && acc_angle < 1.5)
  {
    start = 1;
    gyro_angle = acc_angle;
  }

  // calculate the angle as measured by the gyroscope
  gyro_angle += gyroY_raw * 0.00003051;
  Serial.print("Gyro angle : ");
  Serial.println(gyro_angle);

  // applying the complementary filter to avoid the deviation of angle measured
  if (acc_angle > -5 && acc_angle < 5)
  {
    error = gyro_angle;
  }
  else
  {
    error=(0.9996 * gyro_angle) + (0.0004 * acc_angle);
  }
  Serial.print("error : ");
  Serial.println(error);
  Serial.println((loopTime - micros()));
  Serial.println();

  // PID controller


  while (loopTime > micros());
  loopTime += 4000;
}
