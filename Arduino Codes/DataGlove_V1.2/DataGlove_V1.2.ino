#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include "BluetoothSerial.h"

const char *pin = "1234"; // Change this to more secure PIN.
String device_name = "ESP32-DataGlove";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

using namespace BLA;

int IMUAdress = 0x6A;
int magnetometerAdress = 0x30;
int EEPROMAdress = 0x54;

//finger pings
#define thumb 34
#define index 35
#define middle 32
#define ring 33
#define pinky 27

//finger variables
int thumb_angle = 0;
int index_angle = 0;
int middle_angle = 0;
int ring_angle = 0;
int pinky_angle = 0;

//sensor variables
float magnetometer_cal [3]= {0.0,0.0,0.0};
float accel_cal [3] = {0.0,0.0,0.0};//I call it cal but techniccally it is the raw data, as we do not calibrate it, however it will be easier to understand the madgwick filter if all variables used there are called cal (preventing confusion)
float g [3] = {0.0,0.0,0.0};

//magnetometer corection matrices variables, to be read from EEPROM
typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t; //this should allow me to access the individual float both as a float and as four bytes (the bytes I can the write and read from EEPROM one by one)
float A[3][3];
float b[3];
float A_to_save[9] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};//change these arrays with the values that you want saved
float b_to_save[3] = {30.706940210328740,-41.771984424595686,-3.072929446829696};

//output varaibles
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

//EKF Variables
float q[4] = {1.0,0.0,0.0,0.0}; //initial orientation, doesnt matter cause the filter converts to actaul orientation right away
float P[4][4] = {{0.1,0.0,0.0,0.0},
                 {0.0,0.1,0.0,0.0},
                 {0.0,0.0,0.1,0.0},
                 {0.0,0.0,0.0,0.1}};//currently defined as a full matrix, but once we do efficiecny optimalization, we'll probably find that we dont need to define the full matrix
float R_mag = 2.0;//the actaul estiamtions of R for accelerometer and magnetometer, shouldnt need to be tuned
float R_acc = 0.001;
float Q_val = 0.05;
float R_mag_max = 2.0;//all of the following values are tuable and determine how well the filter will perform
float R_acc_max = 1.0;
float Q_max = 0.1;
float R_mag_min = 0.5;
float R_acc_min = 0.001;
float Q_min = 0.01;
float R_mag_slope = 1.0;
float R_acc_slope = 0.005;
float Q_slope = 0.1;

//Madgwick Variables
float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
float qDot1 = 0.0;
float qDot2 = 0.0;
float qDot3 = 0.0;
float qDot4 = 0.0;

float sampleFrequency = 200.0; //update frequency in Hz
int delayTime = int(1/sampleFrequency);

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  Wire.begin();
  magnetometerSetup();
  IMUSetup();
  //saveMagnetometerMatrices(A_to_save, b_to_save);//matrix A is ROW MAJOR, essentially spred into a noodle
  //delay(1000);
  readMagnetometerMatrices();
  delay(1000);
}

void loop() {
  //magToCSV();//only uncomment in need of calibration of the magnetometer
  collectMagnetometerData();
  collectIMUData();
  EKF();
  //MadgwickFilter();
  //readFingers();
  // writeToMatlab(roll,pitch,yaw);//tried using it for visualization in Matlab but Matlab shits the bed so cant really use it
  printAngles();
  //printAnglesBT();
  //printCalData();
  delay(delayTime);
}

void magnetometerSetup(){
  writeRegister(magnetometerAdress, 0x1C, 2);//configure Internal Control 1
  writeRegister(magnetometerAdress, 0x1A, 208);//configure ODR
  writeRegister(magnetometerAdress, 0x1B, 160);//configure Internal Control 0
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
  float gyro_corr [3] = {0.003418,0.001889,-0.008627};//this is a bias which is measured as the average vlaue of 100+ samples with the sensor at rest

  float accel_raw [3] = {0.0,0.0,0.0};
  
  Wire.beginTransmission(IMUAdress);//start communication with IMU
  Wire.write(0x22);//access first output register (OUTX_L_G)
  Wire.endTransmission();//end communication with slave
  Wire.requestFrom(IMUAdress,12);//request all output bytes (until and including OUTZ_H_A)
  while(Wire.available() < 12);//wait until all bytes are received

  //the output data is in some weird unit, so we multiply by 35 as based on table 3(mechanical characteristics) and the chosen range, and then by 3.14/(180.0*1000.0) to get it from mdps to rad/s
  //35.0*3.14/(180.0*1000.0)=0.000610865
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
  g[0] = gyro_raw[0]*0.000610865-gyro_corr[0];
  g[1] = gyro_raw[1]*0.000610865-gyro_corr[0];
  g[2] = gyro_raw[2]*0.000610865-gyro_corr[0];
}

void MadgwickFilter(){
  float beta = 0.01; //this is the main value we want to tune for the filter, it acts the same way as Kp in a PID controller
  float qDot1 = 0.5*(-q1*(g[0]) - q2*(g[1]) - q3*(g[2]));
  float qDot2 = 0.5*(q0*(g[0]) + q2*(g[2]) - q3*(g[1]));
  float qDot3 = 0.5*(q0*(g[1]) - q1*(g[2]) + q3*(g[0]));
  float qDot4 = 0.5*(q0*(g[2]) + q1*(g[1]) - q2*(g[0]));

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

void EKF(){//assume all angles are in rad, three inputs are the accel, gyro, and amgnetometer data, and then outputs roll pitch yaw, but in itself it wokrs in quaternions
  //PREDICT START
  float mag = sqrt(accel_cal[0]*accel_cal[0] + accel_cal[1]*accel_cal[1] + accel_cal[2]*accel_cal[2]);
  float a[3] = {accel_cal[0]/mag, accel_cal[1]/mag, accel_cal[2]/mag};
  float m[3] = {magnetometer_cal[0], magnetometer_cal[1], magnetometer_cal[2]};

  float q_dot[4] = {0.5*(-q[1]*g[0]-q[2]*g[1]-q[3]*g[2]), 0.5*(q[0]*g[0]-q[3]*g[1]+q[2]*g[2]), 0.5*(q[3]*g[0]+q[0]*g[1]-q[1]*g[2]), 0.5*(-q[2]*g[0]+q[1]*g[1]+q[0]*g[2])};
  for(int i = 0; i < 4; i++){
    q[i] += q_dot[i]/sampleFrequency;
  }
  mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  for(int i = 0; i < 4; i++){
    q[i] /= mag;
  }

  //Estiamte Q based on angulare velocity
  Q_val = Q_max - Q_slope*abs((g[0]+g[1])/2);
  if (Q_val < Q_min){
    Q_val = Q_min;
  }
  float Q[4][4] = {{Q_val, 0.0, 0.0, 0.0},
                   {0.0, Q_val, 0.0, 0.0},
                   {0.0, 0.0, Q_val, 0.0},
                   {0.0, 0.0, 0.0, Q_val}};//Again, mey not need to define full amtrix once I start optimizing the code
  
  float A[4][4] = {{0.0, -g[0]*0.5, -g[1]*0.5, -g[2]*0.5},
                   {g[0]*0.5, 0.0, g[2]*0.5, -g[1]*0.5},
                   {g[1]*0.5, -g[2]*0.5, 0.0, g[0]*0.5},
                   {g[2]*0.5, g[1]*0.5, -g[0]*0.5, 0.5}};

  float temp1[4][4];
  for (int i = 0; i < 4; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 4; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 4; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += A[i][z]*P[z][v];
      }
      temp1[i][v] = inter_result;
    }
  }//multiplication of A[4x4] with P[4x4]

  float A_trans[4][4];
  for (int i = 0; i < 4; i++){//number of collumns of original matrix
    for (int v = 0; v < 4; v++){//number of rows of original matrix 
      A_trans[i][v] = A[v][i];
    }
  }//tranposes 4x4 matrix A

  float temp2[4][4];
  for (int i = 0; i < 4; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 4; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 4; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += P[i][z]*A_trans[z][v];
      }
      temp2[i][v] = inter_result;
    }
  }//multiplication of P[4x4] with A_trans[4x4]

  float temp3[4][4];
  for (int i = 0; i < 4; i++){//number of rows
    for (int v = 0; v < 4; v++){//number of collumns
      temp3[i][v] = (temp1[i][v] + temp2[i][v] + Q[i][v])/sampleFrequency;
    }
  }//sum temp1, temp2, and Q, as well as divide by sampleFrequency

  for (int i = 0; i < 4; i++){//number of rows
    for (int v = 0; v < 4; v++){//number of collumns
      P[i][v] += temp3[i][v];
    }
  }//sum P and temp3
  //PREDICT END
  //CAN REUSE temp1, temp2, temp3, and mag
  //UPDATE START
  float R_to_ref[3][3] = {{q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3], 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2])},
                          {2*(q[1]*q[2]+q[0]*q[3]), q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3], 2*(q[2]*q[3]-q[0]*q[1])},
                          {2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]), q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]}};//cant optimize this as I need to multiply it with with m, so need the full matrix

  float m_prime[3];
  for (int i = 0; i < 3; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 1; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 3; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += R_to_ref[i][z]*m[z];
      }
      m_prime[i] = inter_result;
    }
  }//multiplication of R_to_ref[3x3] with m[3x1], technicaly m is 1x3 but I when you look through the loop it makes sense

  m_prime[2] = 0;
  mag = sqrt(m_prime[0]*m_prime[0]+m_prime[1]*m_prime[1]);
  for(int i = 0; i < 3; i++){
    m_prime[i] /= mag;
  }

  float R_to_body[3][3];//transpose of R_to_ref
  for (int i = 0; i < 3; i++){//number of collumns of original matrix
    for (int v = 0; v < 3; v++){//number of rows of original matrix 
      R_to_body[i][v] = R_to_ref[v][i];
    }
  }//tranposes 3x3 matrix R_to_ref

  float temp4[3];//need this to recalcualte m_prime
  for (int i = 0; i < 3; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 1; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 3; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += R_to_body[i][z]*m_prime[z];
      }
      temp4[i] = inter_result;
    }
  }//multiplication of R_to_body[3x3] with m_prime[3x1], technicaly m_prime is 1x3 but I when ou look through the loop it makes sense
  for(int i = 0; i < 3; i++){
    m_prime[i] = temp4[i];
  }

  // Serial.print(a[0]);Serial.print(" ");
  // Serial.print(a[1]);Serial.print(" ");
  // Serial.print(a[2]);Serial.print(" ");
  // Serial.print(m_prime[0]);Serial.print(" ");
  // Serial.print(m_prime[1]);Serial.print(" ");
  // Serial.print(m_prime[2]);Serial.print(" ");

  float h[6] = {R_to_body[0][2],R_to_body[1][2],R_to_body[2][2],R_to_body[0][1],R_to_body[1][1],R_to_body[2][1]};

  // Serial.print(h[0]);Serial.print(" ");
  // Serial.print(h[1]);Serial.print(" ");
  // Serial.print(h[2]);Serial.print(" ");
  // Serial.print(h[3]);Serial.print(" ");
  // Serial.print(h[4]);Serial.print(" ");
  // Serial.println(h[5]);

  float C[6][4] = {{-q[2]*2, q[3]*2, -q[0]*2, q[1]*2},
                   {q[1]*2, q[0]*2, q[3]*2, q[2]*2},
                   {q[0]*2, -q[1]*2, -q[2]*2, q[3]*2},
                   {q[3]*2, q[2]*2, q[1]*2, q[0]*2},
                   {q[0]*2, -q[1]*2, q[2]*2, -q[3]*2},
                   {-q[1]*2, -q[0]*2, q[3]*2, q[2]*2}};

  //Estimate R based on angular speed of gyroscope
  R_mag = R_mag_slope*abs((g[0]+g[1])/2)+R_mag_min;
  R_acc = R_acc_slope*abs((g[0]+g[1])/2)+R_acc_min;
  if(R_mag > R_mag_max){
    R_mag = R_mag_max;
  }
  if(R_acc > R_acc_max){
    R_acc = R_acc_max;
  }
  float R[6][6] = {{R_acc, 0.0, 0.0, 0.0, 0.0, 0.0},
                   {0.0, R_acc, 0.0, 0.0, 0.0, 0.0},
                   {0.0, 0.0, R_acc, 0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0, R_mag, 0.0, 0.0},
                   {0.0, 0.0, 0.0, 0.0, R_mag, 0.0},
                   {0.0, 0.0, 0.0, 0.0, 0.0, R_mag}};//again, might be able to simplify for optimalization

  float C_trans[4][6];
  for (int i = 0; i < 4; i++){//number of collumns of original matrix
    for (int v = 0; v < 6; v++){//number of rows of original matrix 
      C_trans[i][v] = C[v][i];
    }
  }//tranposes 6x4 matrix C

  float temp5[4][6];
  for (int i = 0; i < 4; i++){//i = number of rows of first matrix
    for (int v = 0; v < 6; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 4; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += P[i][z]*C_trans[z][v];
      }
      temp5[i][v] = inter_result;
    }
  }//multiplication of P[4x4] with C_trans[4x6]

  BLA::Matrix<6, 6> temp6;//I declare this using the LinearAlgebraLibrary so that I can invert it later
  for (int i = 0; i < 6; i++){//i = number of rows of first matrix
    for (int v = 0; v < 6; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 4; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += C[i][z]*temp5[z][v];
      }
      temp6(i,v) = inter_result + R[i][v];
    }
  }//multiplication of C[6x4] with temp5[4x6] and addition of R
  bool is_nonsingular = Invert(temp6);//for this applicaiton, I know the matrix will always be invertible (otherwise we're fucked arent we)
  float K[4][6];
  for (int i = 0; i < 4; i++){//i = number of rows of first matrix
    for (int v = 0; v < 6; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 6; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += temp5[i][z]*temp6(z,v);
      }
      K[i][v] = inter_result;
    }
  }//multiplication of temp5[4x6](P*C_trans) with temp6[6x6](inverted)

  float measurement[6] = {a[0], a[1], a[2], m_prime[0], m_prime[1], m_prime[2]};

  float temp7[4];
  for (int i = 0; i < 4; i++){//i = number of rows of first matrix
    for (int v = 0; v < 1; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 6; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += K[i][z]*(measurement[z]-h[z]);
      }
      temp7[i] = inter_result;
    }
  }//multiplication of K[4x6] with y[6x1](measurement -h), tehcnically it is 1x6 but taken care of in the loop

  for(int i = 0; i < 4; i++){
    q[i] += temp7[i];
  }
  mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  for(int i = 0; i < 4; i++){
    q[i] /= mag;
  }
  //reusing temp1
  for (int i = 0; i < 4; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 4; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 6; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += K[i][z]*C[z][v];
      }
      temp1[i][v] = inter_result;
    }
  }//multiplication of K[4x6] with C[6x4]

  float identity[4][4] = {{1.0, 0.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0, 0.0},
                          {0.0, 0.0, 1.0, 0.0},
                          {0.0, 0.0, 0.0, 1.0}};

  for(int i = 0; i < 4; i++){
    for(int v = 0; v < 4; v++){
      identity[i][v] -= temp1[i][v];
    }
  }

  //reusing temp2
  for (int i = 0; i < 4; i++){//i = nuber of rows of first matrix
    for (int v = 0; v < 4; v++){//v = number of collumns of second matrix
      float inter_result = 0.0;
      for(int z = 0; z < 4; z++){//z = number of collumns of first matrix and number of rows of second matrix 
        inter_result += identity[i][z]*P[z][v];
      }
      temp2[i][v] = inter_result;
    }
  }//multiplication of identity[4x4] with P[4x4]
  
  for(int i = 0; i < 4; i++){
    for(int v = 0; v < 4; v++){
      P[i][v] = temp2[i][v];
    }
  }
  roll = atan2f(q[0]*q[1] + q[2]*q[3], 0.5 - q[1]*q[1] - q[2]*q[2])*180.0/3.14;
  pitch = -asinf(-2.0*(q[1]*q[3] - q[0]*q[2]))*180.0/3.14;
  yaw = -atan2f(q[1]*q[2] + q[0]*q[3], 0.5 - q[2]*q[2] - q[3]*q[3])*180.0/3.14;
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
  Serial.print(g[0]);
  Serial.print("; ");
  Serial.print("Gy: ");
  Serial.print(g[1]);
  Serial.print("; ");
  Serial.print("Gz: ");
  Serial.print(g[2]);
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
  Serial.print("; ");
  Serial.print("Thumb: ");
  Serial.print(thumb_angle);
  Serial.print("; ");
  Serial.print("Index: ");
  Serial.print(index_angle);
  Serial.print("; ");
  Serial.print("Middle: ");
  Serial.print(middle_angle);
  Serial.print("; ");
  Serial.print("Ring: ");
  Serial.print(ring_angle);
  Serial.print("; ");
  Serial.print("Pinky: ");
  Serial.print(pinky_angle);
  Serial.println("; ");
}

void printAnglesBT(){
  SerialBT.print("Roll: ");
  SerialBT.print(roll);
  SerialBT.print("; ");
  SerialBT.print("Pitch: ");
  SerialBT.print(pitch);
  SerialBT.print("; ");
  SerialBT.print("Yaw: ");
  SerialBT.print(yaw);
  SerialBT.print("; ");
  SerialBT.print("Thumb: ");
  SerialBT.print(thumb_angle);
  SerialBT.print("; ");
  SerialBT.print("Index: ");
  SerialBT.print(index_angle);
  SerialBT.print("; ");
  SerialBT.print("Middle: ");
  SerialBT.print(middle_angle);
  SerialBT.print("; ");
  SerialBT.print("Ring: ");
  SerialBT.print(ring_angle);
  SerialBT.print("; ");
  SerialBT.print("Pinky: ");
  SerialBT.print(pinky_angle);
  SerialBT.println("; ");
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

uint8_t readEEPROM(int slaveAdress, int registryAdress){//will only read one register because I ahve to define what I return and I cant felxibly change the length of the return array, could adjust that I always return array of eg 10 elements and only overwrite the number that was requested
  Wire.beginTransmission(slaveAdress);//start communiation with slave   
  //same as for write but should read only the second byte then                                     
  Wire.write(registryAdress-1);//access desired register
  Wire.write(registryAdress);//access desired register
  Wire.endTransmission();
  Wire.requestFrom(slaveAdress,1);
  while(Wire.available() < 1);//wait until all bytes are received
  return Wire.read();
}

void saveMagnetometerMatrices(float A[9], float B[3]){//decompose each float into 4 bytes and save each byte individually in the EEPROM
  for(int i = 0; i < 9; i++){
    FLOATUNION_t tempFloat;
    tempFloat.number = A[i];
    for(int b = 1; b < 5; b++){//EEPROM registries start at 1 (I decided)
      writeEEPROM(EEPROMAdress, i*4+b, tempFloat.bytes[b-1]);
      delay(50);
    }
  }

  for(int i = 0; i < 3; i++){
    FLOATUNION_t tempFloat;
    tempFloat.number = B[i];
    for(int b = 1; b < 5; b++){//EEPROM registries start at 1 (I decided)
      writeEEPROM(EEPROMAdress, 36+i*4+b, tempFloat.bytes[b-1]);//the first 36 bytes are used for matrix A
      delay(50);
    }
  }
}

void readMagnetometerMatrices(){
  for(int i = 0; i < 9; i++){
    FLOATUNION_t tempFloat;
    for(int b = 1; b < 5; b++){//EEPROM registries start at 1 (I decided)
      tempFloat.bytes[b-1] = readEEPROM(EEPROMAdress, i*4+b);
      delay(50);
    }
    A[int(i/3)][i%3] = tempFloat.number;
  }

  for(int i = 0; i < 3; i++){
    FLOATUNION_t tempFloat;
    for(int b = 1; b < 5; b++){//EEPROM registries start at 1 (I decided)
      tempFloat.bytes[b-1] = readEEPROM(EEPROMAdress, 36+i*4+b);//the first 36 bytes are used for matrix A
      delay(50);
    }
    b[i] = tempFloat.number;
  }
}

void readFingers(){
  thumb_angle = map(analogRead(thumb),890,1750,0,75);
  index_angle = map(analogRead(index),890,1750,0,75);
  middle_angle = map(analogRead(middle),890,1750,0,75);
  ring_angle = map(analogRead(ring),890,1750,0,75);
  pinky_angle = map(analogRead(pinky),890,1750,0,75);
}

void magToCSV(){//this function is used to output magdata such that it can be written into a csv file (need arduino 1.8 on lapotpo with that special addon), the result can be used in matlab script whihc then genrates the calibration amtrices that canbe copeid here
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
  Serial.print(magnetometer_raw[0]);Serial.print(",");
  Serial.print(magnetometer_raw[1]);Serial.print(",");
  Serial.println(magnetometer_raw[2]);
}

void writeToMatlab(float number1, float number2, float number3){
  byte *b = (byte *) &number1;
  Serial.write(b,4);
  b = (byte *) &number2;
  Serial.write(b,4);
  b = (byte *) &number3;
  Serial.write(b,4);
  Serial.write("\n");// terminator selected in matlab
}
