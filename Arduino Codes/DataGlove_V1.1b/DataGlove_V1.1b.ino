#include <Wire.h>

int IMUAdress = 0x6A;
int magnetometerAdress = 0x30;
int EEPROMAdress = 0x54;

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

float sampleFrequency = 200.0; //update frequency in Hz
int delayTime = int(1/sampleFrequency);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  magnetometerSetup();
  IMUSetup();
}

void loop() {
  collectMagnetometerData();
  collectIMUData();
  MadgwickFilter();
  printAngles();
  //printCalData();
  delay(delayTime);
}

void magnetometerSetup(){
  writeRegister(magnetometerAdress, 0x1A, 208);//configure ODR
  writeRegister(magnetometerAdress, 0x1B, 128);//configure Internal Control 0
  writeRegister(magnetometerAdress, 0x1C, 2);//configure Internal Control 1
  writeRegister(magnetometerAdress, 0x1D, 16);//configure Internal Control 2
}

void IMUSetup(){
  writeRegister(IMUAdress, 0x10, 90);//configure CTRL1_XL
  writeRegister(IMUAdress, 0x11, 88);//configure CTRL2_G
  writeRegister(IMUAdress, 0x12, 4);//configure CTRL3_C
  writeRegister(IMUAdress, 0x13, 2);//configure CTRL4_C
  writeRegister(IMUAdress, 0x15, 0);//configure CTRL6_C
  writeRegister(IMUAdress, 0x17, 0);//configure CTRL8_XL
  writeRegister(IMUAdress, 0x18, 226);//configure CTRL9_XL
}

void collectMagnetometerData(){
  float A [3][3] = {{1.036636467627914,0.015579013134350,-0.027268011380725},
                    {0.015579013134350,0.998173875917809,-0.105352624885353},
                    {-0.027268011380725,-0.105352624885353,0.978402869105677}};
  float b [3] = {33.588119046293370,21.605057904894150,-98.617904339319760};
  float magnetometer_raw [3] = {0.0,0.0,0.0};
  uint8_t buffer [9] = {0,0,0,0,0,0,0,0,0};
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;

  Wire.beginTransmission(magnetometerAdress);//start communication with magnetometer
  Wire.write(0x00);//access first output register (Xout0)
  Wire.endTransmission();//end communication with slave
  Wire.requestFrom(magnetometerAdress,9);//request all output bytes (until and including OUTZ_H_A)
  while(Wire.available() < 9);//wait until all bytes are received
  for (int i = 0; i < 9; i++){//feed all data into the buffer array
    buffer[i] = Wire.read();
  }

  x = (uint32_t)buffer[0] << 12 | (uint32_t)buffer[1] << 4 | (uint32_t)buffer[6] >> 4;//piece together all the bytes corresponding to 1 axis
  y = (uint32_t)buffer[2] << 12 | (uint32_t)buffer[3] << 4 | (uint32_t)buffer[7] >> 4;
  z = (uint32_t)buffer[4] << 12 | (uint32_t)buffer[5] << 4 | (uint32_t)buffer[8] >> 4;

  x -= (uint32_t)1 << 19;//fix center offsets(something to do with the fact data is sent unsigned but we need to make it singed, I think)
  y -= (uint32_t)1 << 19;
  z -= (uint32_t)1 << 19;

  magnetometer_raw[0] = float(x)*0.00625;//convert from count directly to uT (based on datasheet)
  magnetometer_raw[1] = float(y)*0.00625;
  magnetometer_raw[2] = float(z)*0.00625;
  //correct for soft and hard irons
  float magnetometer_corr [3] = {magnetometer_raw[0]-b[0], magnetometer_raw[1]-b[1], magnetometer_raw[2]-b[2]};
  magnetometer_cal[0] = {magnetometer_corr[0]*A[0][0]+magnetometer_corr[1]*A[1][0]+magnetometer_corr[2]*A[2][0]};
  magnetometer_cal[1] = {magnetometer_corr[0]*A[0][1]+magnetometer_corr[1]*A[1][1]+magnetometer_corr[2]*A[2][1]};
  magnetometer_cal[2] = {magnetometer_corr[0]*A[0][2]+magnetometer_corr[1]*A[1][2]+magnetometer_corr[2]*A[2][2]};
}

void collectIMUData(){
  float gyro_raw [3] = {0.0,0.0,0.0};
  float gyro_corr [3] = {0.003418,0.001889,-0.008627};
  
  Wire.beginTransmission(IMUAdress);//start communication with IMU
  Wire.write(0x22);//access first output register (OUTX_L_G)
  Wire.endTransmission();//end communication with slave
  Wire.requestFrom(IMUAdress,12);//request all output bytes (until and including OUTZ_H_A)
  while(Wire.available() < 12);//wait until all bytes are received

  gyro_raw[0] = float(int16_t(Wire.read()|(Wire.read()<<8)));//read first byte (low byte of GYRO_X) and combine it with the second byte (high byte of GYRO_X) that has been shifted to the left), so on for the other variables
  gyro_raw[1] = float(int16_t(Wire.read()|(Wire.read()<<8)));//gotta make sure the two bytes read are interpreted correctly as a signed 16 bit integer and then we convert to float
  gyro_raw[2] = float(int16_t(Wire.read()|(Wire.read()<<8)));

  //the output data is in some weird unit, so we multiply by 0.244 as based on table 3(mechanical characteristics) and the chosen range, and then by 9.81/1000 to trasnfer from mg to m/s^2
  //0.244*9.81/1000.0=0.00239364
  accel_cal[0] = float(int16_t(Wire.read()|(Wire.read()<<8)))*0.00239364;
  accel_cal[1] = float(int16_t(Wire.read()|(Wire.read()<<8)))*0.00239364;
  accel_cal[2] = float(int16_t(Wire.read()|(Wire.read()<<8)))*0.00239364;  

  //the output data is in some weird unit, so we multiply by 35 as based on table 3(mechanical characteristics) and the chosen range, and then by 3.14/(180.0*1000.0) to get it from mdps to rad/s
  //35.0*3.14/(180.0*1000.0)=0.000610865
  gyro_cal[0] = gyro_raw[0]*0.000610865-gyro_corr[0];
  gyro_cal[1] = gyro_raw[1]*0.000610865-gyro_corr[0];
  gyro_cal[2] = gyro_raw[2]*0.000610865-gyro_corr[0];
}

void MadgwickFilter(){
  float beta = 0.01; //this is the main value we want to tune for the filter, it acts the same way as Kp in a PID controller
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

  roll = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2)*180.0/3.14;
  pitch = asinf(-2.0*(q1*q3 - q0*q2))*180.0/3.14;
  yaw = atan2f(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3)*180.0/3.14;
}

void printCalData(){
  Serial.print("Mx: ");
  Serial.print(magnetometer_cal[0]);
  Serial.print("; ");
  Serial.print("My: ");
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

void writeRegister(int slaveAdress, int registryAdress, int desiredValue){
  Wire.beginTransmission(slaveAdress);//start communiation with slave                                        
  Wire.write(registryAdress);//access desired register
  Wire.write(desiredValue);//set the new registry value                                                   
  Wire.endTransmission();//end communication with slave  
}

void writeEEPROM(int slaveAdress, int registryAdress, int desiredValue){//write one byte into the desired registry, CANNOT WRITE INTO REGITRY 0!!!!!
  if(registryAdress <= 0){
    registryAdress = 1;//if bad registry adress given, will write into 
  }
  Wire.beginTransmission(slaveAdress);//start communiation with slave 
  //IDK why, but the AT24C256C needs to be given two registry adress (2 bytes) but only one byte of data is sent, so I am asumming it gets written into the second byte adress and so only the second adress byte gets affected                                       
  Wire.write(registryAdress-1);//access desired register
  Wire.write(registryAdress);//set the new registry value
  Wire.write(desiredValue);                                                   
  Wire.endTransmission();//end communication with slave  
}

int readEEPROM(int slaveAdress, int registryAdress){//will only read one register because I ahve to define what I return and I cant felxibly change the length of the return array, could adjust that I always return array of eg 10 elements and only overwrite the number that was requested
  Wire.beginTransmission(slaveAdress);//start communiation with slave   
  //same as for write but should read only the second byte then                                     
  Wire.write(registryAdress-1);//access desired register
  Wire.write(registryAdress);//access desired register
  Wire.endTransmission();
  Wire.requestFrom(slaveAdress,1);
  while(Wire.available() < 1);//wait until all bytes are received
  return Wire.read();
}