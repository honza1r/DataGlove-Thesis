#include <Adafruit_MMC56x3.h>
#include <Wire.h>

//currently, nothing AINT WORKING MUDAFUKA!!!!!
Adafruit_MMC5603 magnetometer = Adafruit_MMC5603(12345);

float magnetometer_cal [3]= {0.0,0.0,0.0};
float accel_cal [3] = {0.0,0.0,0.0};//I call it cal but techniccally it is the raw data, as we do not calibrate it, however it will be easier to understand the madgwick filter if all variables used there are called cal (preventing confusion)
float gyro_cal [3] = {0.0,0.0,0.0};

float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
float qDot1 = 0.0;
float qDot2 = 0.0;
float qDot3 = 0.0;
float qDot4 = 0.0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //magnetometerSetup();
  IMUSetup();
}

void loop() {
  //collectMagnetometerData();
  collectIMUData();
  printCalData();
  //MadgwickFilter();
  //printAngles();
  delay(5);
}

void magnetometerSetup(){
  magnetometer.begin(MMC56X3_DEFAULT_ADDRESS, &Wire);
  magnetometer.setDataRate(220);
  magnetometer.setContinuousMode(true);
}

void IMUSetup(){
  //configure accelerometer rate and range; 208HZ,8G,Digital low pass filter (LPF2) enable
  Wire.beginTransmission(0x6A);//LSM6DS32 I2C adress
  Wire.write(0x10);//register address,CTRL_1_XL
  Wire.write(0x5A);//register value
  Wire.endTransmission();
  //configure the digital low pass filter (LPF2); set to ODR/100,
  Wire.beginTransmission(0x6A);//LSM6DS32 I2C adress
  Wire.write(0x17);//register address,CTRL8_XL
  Wire.write(0x80);//register value
  Wire.endTransmission();
  //configure gyroscope rate and range; 208Hz,1000DPS
  Wire.beginTransmission(0x6A);//LSM6DS32 I2C adress
  Wire.write(0x11);//register address,CTRL8_XL
  Wire.write(0x58);//register value
  Wire.endTransmission();
  //configure the first low pass filter (LPF1); TRIG_EN=1, FTYPE (the actaul bandwidth setting)=12.2Hz
  Wire.beginTransmission(0x6A);//LSM6DS32 I2C adress
  Wire.write(0x15);//register address,CTRL6_C
  Wire.write(0x87);//register value
  Wire.endTransmission();
}

void collectMagnetometerData(){
  float A [3][3] = {{1.036636467627914,0.015579013134350,-0.027268011380725},
                    {0.015579013134350,0.998173875917809,-0.105352624885353},
                    {-0.027268011380725,-0.105352624885353,0.978402869105677}};
  float b [3] = {33.588119046293370,21.605057904894150,-98.617904339319760};
  sensors_event_t event_magnetometer;
  magnetometer.getEvent(&event_magnetometer);
  float magnetometer_raw [3] = {event_magnetometer.magnetic.x,event_magnetometer.magnetic.y,event_magnetometer.magnetic.z};
  float magnetometer_corr [3] = {magnetometer_raw[0]-b[0], magnetometer_raw[1]-b[1], magnetometer_raw[2]-b[2]};
  magnetometer_cal[0] = {magnetometer_corr[0]*A[0][0]+magnetometer_corr[1]*A[1][0]+magnetometer_corr[2]*A[2][0]};
  magnetometer_cal[1] = {magnetometer_corr[0]*A[0][1]+magnetometer_corr[1]*A[1][1]+magnetometer_corr[2]*A[2][1]};
  magnetometer_cal[2] = {magnetometer_corr[0]*A[0][2]+magnetometer_corr[1]*A[1][2]+magnetometer_corr[2]*A[2][2]};
}

void collectIMUData(){
  float gyro_raw [3] = {0.0,0.0,0.0};
  float gyro_corr [3] = {0.003418,0.001889,-0.008627};
  //collect values from IMU
  Wire.beginTransmission(0x6A);                                       
  Wire.write(0x23);//start from gyroscope registers                                                 
  Wire.endTransmission();
  Wire.requestFrom(0x6A,12);//request the next 12 bytes (not reading temp)
  while(Wire.available() < 12);
  gyro_raw[0] = Wire.read()<<8|Wire.read();//every two bytes are different axis values
  gyro_raw[1] = Wire.read()<<8|Wire.read();
  gyro_raw[2] = Wire.read()<<8|Wire.read();
  accel_cal[0] = Wire.read()<<8|Wire.read();
  accel_cal[1] = Wire.read()<<8|Wire.read();
  accel_cal[2] = Wire.read()<<8|Wire.read();  
  gyro_cal[0] = gyro_raw[0]-gyro_corr[0];
  gyro_cal[1] = gyro_raw[1]-gyro_corr[1];
  gyro_cal[2] = gyro_raw[2]-gyro_corr[2];
}

void MadgwickFilter(){
  float beta = 0.1; //this is the main value we want to tune for the filter, it acts the same way as Kp in a PID controller
  float sampleFrequency = 200.0; //update frequency in Hz
  float qDot1 = 0.5*(-q1*(gyro_cal[0]) - q2*(gyro_cal[1]) - q3*(gyro_cal[2]));
  float qDot2 = 0.5*(q0*(gyro_cal[0]) + q2*(gyro_cal[2]) - q3*(gyro_cal[1]));
  float qDot3 = 0.5*(q0*(gyro_cal[1]) - q1*(gyro_cal[2]) + q3*(gyro_cal[0]));
  float qDot4 = 0.5*(q0*(gyro_cal[2]) + q1*(gyro_cal[1]) - q2*(gyro_cal[0]));

  float recipNorm = 0.0;

  if (!((accel_cal[0] == 0.0) && (accel_cal[1] == 0.0) && (accel_cal[2] == 0.0))){//ensures we only compute update if accelerometer readings are valid
    //normalize accelerometer readings
    recipNorm = 1/(sqrt(accel_cal[0]*accel_cal[0]+accel_cal[1]*accel_cal[1]+accel_cal[2]*accel_cal[2]));
    float ax = accel_cal[0]*recipNorm;
    float ay = accel_cal[1]*recipNorm;
    float az = accel_cal[2]*recipNorm;
    //normalize magnetometer readings
    recipNorm = 1/(sqrt(magnetometer_cal[0]*magnetometer_cal[0]+magnetometer_cal[1]*magnetometer_cal[1]+magnetometer_cal[2]*magnetometer_cal[2]));
    float mx = magnetometer_cal[0]*recipNorm;
    float my = magnetometer_cal[1]*recipNorm;
    float mz = magnetometer_cal[2]*recipNorm;
    //the following varaibles are "pre-calculated" once so we save some computatonal time by not doing the calcualtions over and over again when they are needed
    float twoq0mx = 2.0*q0*mx;
    float twoq0my = 2.0*q0*my;
    float twoq0mz = 2.0*q0*mz;

    float twoq1mx = 2.0*q1*mx;

    float twoq0 = 2.0*q0;
    float twoq1 = 2.0*q1;
    float twoq2 = 2.0*q2;
    float twoq3 = 2.0*q3;

    float twoq0q2 = 2.0*q0*q2;
    float twoq2q3 = 2.0*q2*q3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    //reference direction of Earth's magnetic field
    float hx = mx*q0q0 - twoq0my*q3 + twoq0mz*q2 + mx*q1q1 + twoq1*my*q2 + twoq1*mz*q3 - mx*q2q2 - mx*q3q3;
    float hy = twoq0mx*q3 + my*q0q0 - twoq0mz*q1 + twoq1mx*q2 - my*q1q1 + my*q2q2 + twoq2*mz*q3 - my*q3q3;

    float twobx = sqrt(hx*hx + hy*hy);
    float twobz = -twoq0mx*q2 + twoq0my*q1 + mz*q0q0 + twoq1mx*q3 - mz*q1q1 + twoq2*my*q3 - mz*q2q2 + mz*q3q3;
    float fourbx = 2.0*twobx;
    float fourbz = 2.0*twobz;

    float s0 = -twoq2 * (2.0 * q1q3 - twoq0q2 - ax) + twoq1 * (2.0 * q0q1 + twoq2q3 - ay) - twobz * q2 * (twobx * (0.5 - q2q2 - q3q3) + twobz * (q1q3 - q0q2) - mx) + (-twobx * q3 + twobz * q1) * (twobx * (q1q2 - q0q3) + twobz * (q0q1 + q2q3) - my) + twobx * q2 * (twobx * (q0q2 + q1q3) + twobz * (0.5 - q1q1 - q2q2) - mz);
    float s1 = s1 = twoq3 * (2.0 * q1q3 - twoq0q2 - ax) + twoq0 * (2.0 * q0q1 + twoq2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + twobz * q3 * (twobx * (0.5 - q2q2 - q3q3) + twobz * (q1q3 - q0q2) - mx) + (twobx * q2 + twobz * q0) * (twobx * (q1q2 - q0q3) + twobz * (q0q1 + q2q3) - my) + (twobx * q3 - fourbz * q1) * (twobx * (q0q2 + q1q3) + twobz * (0.5 - q1q1 - q2q2) - mz);
    float s2 = -twoq0 * (2.0 * q1q3 - twoq0q2 - ax) + twoq3 * (2.0 * q0q1 + twoq2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-fourbx * q2 - twobz * q0) * (twobx * (0.5 - q2q2 - q3q3) + twobz * (q1q3 - q0q2) - mx) + (twobx * q1 + twobz * q3) * (twobx * (q1q2 - q0q3) + twobz * (q0q1 + q2q3) - my) + (twobx * q0 - fourbz * q2) * (twobx * (q0q2 + q1q3) + twobz * (0.5 - q1q1 - q2q2) - mz);
    float s3 = s3 = twoq1 * (2.0 * q1q3 - twoq0q2 - ax) + twoq2 * (2.0 * q0q1 + twoq2q3 - ay) + (-fourbx * q3 + twobz * q1) * (twobx * (0.5 - q2q2 - q3q3) + twobz * (q1q3 - q0q2) - mx) + (-twobx * q0 + twobz * q2) * (twobx * (q1q2 - q0q3) + twobz * (q0q1 + q2q3) - my) + twobx * q1 * (twobx * (q0q2 + q1q3) + twobz * (0.5 - q1q1 - q2q2) - mz);

    recipNorm = 1/sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta*s0;
    qDot2 -= beta*s1;
    qDot3 -= beta*s2;
    qDot4 -= beta*s3;

    roll = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2)*180.0/3.14;
    pitch = asinf(-2.0*(q1*q3 - q0*q2))*180.0/3.14;
    yaw = atan2f(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3)*180.0/3.14;
  }

  q0 += qDot1/sampleFrequency;
  q1 += qDot2/sampleFrequency;
  q2 += qDot3/sampleFrequency;
  q3 += qDot4/sampleFrequency;

  recipNorm = 1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void printCalData(){
  Serial.print("Mx: ");
  Serial.print(magnetometer_cal[0]);
  Serial.print("; ");
  Serial.print("y: ");
  Serial.print(magnetometer_cal[1]);
  Serial.print("; ");
  Serial.print("Mz: ");
  Serial.print(magnetometer_cal[2]);
  Serial.print("; ");
  Serial.print("Ax: ");
  Serial.print(accel_cal[0]);
  Serial.print("; ");
  Serial.print("Ay: ");
  Serial.print(accel_cal[1]);
  Serial.print("; ");
  Serial.print("Az: ");
  Serial.print(accel_cal[2]);
  Serial.print("; ");
  Serial.print("Gx: ");
  Serial.print(gyro_cal[0]);
  Serial.print("; ");
  Serial.print("Gy: ");
  Serial.print(gyro_cal[1]);
  Serial.print("; ");
  Serial.print("Gz: ");
  Serial.print(gyro_cal[2]);
  Serial.println("; ");
}

void printAngles(){
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("; ");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("; ");
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.println("; ");
}