#include <Wire.h>
#include <MatrixMath.h>

int IMUAdress = 0x6A;
int magnetometerAdress = 0x30;

float magnetometer_cal [3]= {0.0,0.0,0.0};
float accel_cal [3] = {0.0,0.0,0.0};//I call it cal but techniccally it is the raw data, as we do not calibrate it, however it will be easier to understand the madgwick filter if all variables used there are called cal (preventing confusion)
float gyro_cal [3] = {0.0,0.0,0.0};

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

//some constant that may or may not require tuning
float p_init_constant = 0.01;
float Q_init_constant = 0.001;
float R_init_constant = 0.1;

//EKF initial values
float qua[4] = {1.0, 0.0, 0.0, 0.0};//1x4 matrix
float bias[3] = {0.0, 0.0, 0.0};//1x3 matrix

float xHat[7] = {qua[0], 
                 qua[1], 
                 qua[2], 
                 qua[3], 
                 bias[0], 
                 bias[1], 
                 bias[2]}; //q and bias concatenated and transposed, 7x1 matrix

float xHatPrev[7];//7x1 matrix

float xHatBar[7];//7x1 matrix

float yHatBar[6];//6x1 matrix

float p[7][7] = {{1.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 1.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 0.0*p_init_constant, 1.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 1.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 1.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 1.0*p_init_constant, 0.0*p_init_constant},
                 {0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 0.0*p_init_constant, 1.0*p_init_constant}};//7x7 identity matrix multiplied by the init constant

float Q[7][7] = {{1.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 1.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 0.0*Q_init_constant, 1.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 1.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 1.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 1.0*Q_init_constant, 0.0*Q_init_constant},
                 {0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 0.0*Q_init_constant, 1.0*Q_init_constant}};//7x7 identity matrix multiplied by the init constant

float R[6][6] = {{1.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant},
                 {0.0*R_init_constant, 1.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant},
                 {0.0*R_init_constant, 0.0*R_init_constant, 1.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant},
                 {0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 1.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant},
                 {0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 1.0*R_init_constant, 0.0*R_init_constant},
                 {0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 0.0*R_init_constant, 1.0*R_init_constant}};//7x7 identity matrix multiplied by the init constant
//there are some variables that are simply declared as None in python so I will have to declare them once I know what type of variable they are
float A[7][7];
float B[7][3];
float C[6][7];
float pBar[7][7];
float K[7][6];
float accelReference[3] = {0.0,
                           0.0,
                          -1.0};//3x1 matrix

float magReference[3] = {0.0,
                        -1.0,
                         0.0};//3x1 matrix

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
  EKF();
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
  gyro_cal[0] = gyro_raw[0]*0.000610865-gyro_corr[0];
  gyro_cal[1] = gyro_raw[1]*0.000610865-gyro_corr[0];
  gyro_cal[2] = gyro_raw[2]*0.000610865-gyro_corr[0];

  //dont need another set of arrays to store the previous value as the previous value is the last cal value, so I just use it to recalculate the new one

  // //0.01 for alpha is taken bsed on tutorial, can be tuned as needed. This whole filter step can be removed or kept based on results!!!!
  // gyro_cal[0] = IIR_filter(0.01, gyro_cal[0], gyro_raw[0]);
  // gyro_cal[1] = IIR_filter(0.01, gyro_cal[1], gyro_raw[1]);
  // gyro_cal[2] = IIR_filter(0.01, gyro_cal[2], gyro_raw[2]);

  // //0.1 for alpha is taken bsed on tutorial, can be tuned as needed. This whole filter step can be removed or kept based on results!!!!
  // accel_cal[0] = IIR_filter(0.1, accel_cal[0], accel_raw[0]);
  // accel_cal[1] = IIR_filter(0.1, accel_cal[1], accel_raw[1]);
  // accel_cal[2] = IIR_filter(0.1, accel_cal[2], accel_raw[2]);
}

float IIR_filter(float alpha, float prevVal, float measuredVal){
  return alpha * prevVal + (1-alpha) * measuredVal;
}

//the EKF has two major repeated steps, prediction (calcualtion based on gyroscope) and update (based on accel/magnetometer), and essentially we calcualte angle based on gyroscope and every once in a while we correct for the gyro drift with accel/magnetometer
//it is essentially an advanced complimentary filter, theoretically, each step can run at  a difference frequency (based on how fast different sensors are) but since all of my sensors are capable of the same update speed then I will always perform both steps
//at the same time, which should actaully increase accuracy
void EKF(){//assume all angles are in rad
  //PREDICT
  float q[4] = {xHat[0], xHat[1], xHat[2], xHat[3]}; 
  float Sq[4][3] = {{-q[1], -q[2], -q[3]},
                    {q[0], -q[3], q[2]},
                    {q[3], q[0], -q[1]},
                    {-q[2], q[1], q[0]}};
  float Sq1[4][3];
  float Sq2[4][3];
  for (int i = 0; i < 4; i++){
    for (int v = 0; v < 3; v++){
      Sq1[i][v] = Sq[i][v]*(-delayTime/2);
    }
  }
  for (int i = 0; i < 4; i++){
    for (int v = 0; v < 3; v++){
      Sq2[i][v] = Sq[i][v]*(delayTime/2);
    }
  }
  float temp1[4][7] = {{1.0, 0.0, 0.0, 0.0, Sq1[0][0], Sq1[0][1], Sq1[0][2]},
                       {0.0, 1.0, 0.0, 0.0, Sq1[1][0], Sq1[1][1], Sq1[1][2]},
                       {0.0, 0.0, 1.0, 0.0, Sq1[2][0], Sq1[2][1], Sq1[2][2]},
                       {0.0, 0.0, 0.0, 1.0, Sq1[3][0], Sq1[3][1], Sq1[3][2]}};//concentate a 4x4 identity matrix with Sq
  float temp2[3][7] = {{0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                       {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},};//concentate a 3x4 zeros matrix with 3x3 identity matrix
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 4; v++){
      A[v][i] = temp1[v][i];
    }
    for (int y = 4; y < 7; y++){
      A[y][i] = temp2[y-4][i];
    }
  }//concentate temp1 and temp2 vertiaclly
  for (int i = 0; i < 3; i++){
    for (int v = 0; v < 4; v++){
      B[v][i] = Sq2[v][i];
    }
    for (int y = 4; y < 7; y++){
      B[y][i] = 0.0;
    }
  }//concentate Sq2 with a 3x3 zeroes matrix vertically

  float helper1[7];//7x1 matrix
  for (int i = 0; i < 7; i++){
    float row_result = 0.0;
    for (int v = 0; v < 7; v++){
      row_result = row_result + A[i][v]*xHat[v];
    }
    helper1[i] = row_result;
  }//7x7 multiplication with 7x1

  float helper2[7];//7x1 matrix
  for (int i = 0; i < 7; i++){
    float row_result = 0.0;
    for (int v = 0; v < 3; v++){
      row_result = row_result + B[i][v]*gyro_cal[v];//THIS IS WHERE GYRO DATA APPEARS!!!
    }
    helper2[i] = row_result;
  }//7x3 multiplication with 3x1

  for (int i = 0; i < 7; i++){
    xHatBar[i] = helper1[i] + helper2[i];
  }//sum helper1 with helper2

  float quatMag =sqrt(xHatBar[0]*xHatBar[0] + xHatBar[1]*xHatBar[1] + xHatBar[2]*xHatBar[2] + xHatBar[3]*xHatBar[3]);
  for (int i = 0; i < 4; i++){
    xHatBar[i] = xHatBar[i]/quatMag;
  }//normalize quaternion in xHatBar

  for (int i = 0; i < 7; i++){
    xHatPrev[i] = xHat[i];
  }

  //THE next bit is predictAccelMag
  float qHatPrev[4] = {xHatPrev[0], xHatPrev[1], xHatPrev[2], xHatPrev[3]};
  float e00 = 2*(qHatPrev[0] * accelReference[0] + qHatPrev[3] * accelReference[1] - qHatPrev[2] * accelReference[2]);
  float e01 = 2*(qHatPrev[1] * accelReference[0] + qHatPrev[2] * accelReference[1] + qHatPrev[3] * accelReference[2]);
  float e02 = 2*(-qHatPrev[2] * accelReference[0] + qHatPrev[1] * accelReference[1] - qHatPrev[0] * accelReference[2]);
  float e03 = 2*(-qHatPrev[3] * accelReference[0] + qHatPrev[0] * accelReference[1] + qHatPrev[1] * accelReference[2]);
  float e10 = 2*(-qHatPrev[3] * accelReference[0] + qHatPrev[0] * accelReference[1] + qHatPrev[1] * accelReference[2]);
  float e11 = 2*(qHatPrev[2] * accelReference[0] - qHatPrev[1] * accelReference[1] + qHatPrev[0] * accelReference[2]);
  float e12 = 2*(qHatPrev[1] * accelReference[0] + qHatPrev[2] * accelReference[1] + qHatPrev[3] * accelReference[2]);
  float e13 = 2*(-qHatPrev[0] * accelReference[0] - qHatPrev[3] * accelReference[1] + qHatPrev[2] * accelReference[2]);
  float e20 = 2*(qHatPrev[2] * accelReference[0] - qHatPrev[1] * accelReference[1] + qHatPrev[0] * accelReference[2]);
  float e21 = 2*(qHatPrev[3] * accelReference[0] - qHatPrev[0] * accelReference[1] - qHatPrev[1] * accelReference[2]);
  float e22 = 2*(qHatPrev[0] * accelReference[0] + qHatPrev[3] * accelReference[1] - qHatPrev[2] * accelReference[2]);
  float e23 = 2*(qHatPrev[1] * accelReference[0] + qHatPrev[2] * accelReference[1] + qHatPrev[3] * accelReference[2]);
  float hPrime_a[3][4] ={{e00, e01, e02, e03},
                         {e10, e11, e12, e13},
                         {e20, e21, e22, e23}};//Jacobian matrix based on acceleration reference (smth like that?)

  //following values are for a rotational matrix
  float c00 = xHatBar[0] * xHatBar[0] + xHatBar[1] * xHatBar[1] - xHatBar[2] * xHatBar[2] - xHatBar[3] * xHatBar[3];
  float c01 = 2 * (xHatBar[1] * xHatBar[2] - xHatBar[0] * xHatBar[3]);
  float c02 = 2 * (xHatBar[1] * xHatBar[3] + xHatBar[0] * xHatBar[2]);
  float c10 = 2 * (xHatBar[1] * xHatBar[2] + xHatBar[0] * xHatBar[3]);
  float c11 = xHatBar[0] * xHatBar[0] - xHatBar[1] * xHatBar[1] + xHatBar[2] * xHatBar[2] - xHatBar[3] * xHatBar[3];
  float c12 = 2 * (xHatBar[2] * xHatBar[3] - xHatBar[0] * xHatBar[1]);
  float c20 = 2 * (xHatBar[1] * xHatBar[3] - xHatBar[0] * xHatBar[2]);
  float c21 = 2 * (xHatBar[2] * xHatBar[3] + xHatBar[0] * xHatBar[1]);
  float c22 = xHatBar[0] * xHatBar[0] - xHatBar[1] * xHatBar[1] - xHatBar[2] * xHatBar[2] + xHatBar[3] * xHatBar[3];
  
  float tempo1[3][3] = {{c00, c10, c20},
                        {c01, c11, c21},
                        {c02, c12, c22}};//3x3 matrix that is the transposed rotational matrix

  float accelBar[3];//3x1 matrix
  for (int i = 0; i < 3; i++){
    float row_result = 0.0;
    for (int v = 0; v < 3; v++){
      row_result = row_result + tempo1[i][v]*accelReference[v];
    }
    accelBar[i] = row_result;
  }//multiplication of tempo1[3x3] with accelReference[3x1]

  e00 = 2*(qHatPrev[0] * magReference[0] + qHatPrev[3] * magReference[1] - qHatPrev[2] * magReference[2]);
  e01 = 2*(qHatPrev[1] * magReference[0] + qHatPrev[2] * magReference[1] + qHatPrev[3] * magReference[2]);
  e02 = 2*(-qHatPrev[2] * magReference[0] + qHatPrev[1] * magReference[1] - qHatPrev[0] * magReference[2]);
  e03 = 2*(-qHatPrev[3] * magReference[0] + qHatPrev[0] * magReference[1] + qHatPrev[1] * magReference[2]);
  e10 = 2*(-qHatPrev[3] * magReference[0] + qHatPrev[0] * magReference[1] + qHatPrev[1] * magReference[2]);
  e11 = 2*(qHatPrev[2] * magReference[0] - qHatPrev[1] * magReference[1] + qHatPrev[0] * magReference[2]);
  e12 = 2*(qHatPrev[1] * magReference[0] + qHatPrev[2] * magReference[1] + qHatPrev[3] * magReference[2]);
  e13 = 2*(-qHatPrev[0] * magReference[0] - qHatPrev[3] * magReference[1] + qHatPrev[2] * magReference[2]);
  e20 = 2*(qHatPrev[2] * magReference[0] - qHatPrev[1] * magReference[1] + qHatPrev[0] * magReference[2]);
  e21 = 2*(qHatPrev[3] * magReference[0] - qHatPrev[0] * magReference[1] - qHatPrev[1] * magReference[2]);
  e22 = 2*(qHatPrev[0] * magReference[0] + qHatPrev[3] * magReference[1] - qHatPrev[2] * magReference[2]);
  e23 = 2*(qHatPrev[1] * magReference[0] + qHatPrev[2] * magReference[1] + qHatPrev[3] * magReference[2]);
  float hPrime_m[3][4] ={{e00, e01, e02, e03},
                         {e10, e11, e12, e13},
                         {e20, e21, e22, e23}};//Jacobian matrix based on magnetometer reference (smth like that?)

  float magBar[3];//3x1 matrix
  for (int i = 0; i < 3; i++){
    float row_result = 0.0;
    for (int v = 0; v < 3; v++){
      row_result = row_result + tempo1[i][v]*magReference[v];
    }
    magBar[i] = row_result;
  }//multiplication of tempo1[3x3] with magReference[3x1]

  float tmp1[3][7] = {{hPrime_a[0][0], hPrime_a[0][1], hPrime_a[0][2], hPrime_a[0][3], 0.0, 0.0, 0.0},
                      {hPrime_a[1][0], hPrime_a[1][1], hPrime_a[1][2], hPrime_a[1][3], 0.0, 0.0, 0.0},
                      {hPrime_a[2][0], hPrime_a[2][1], hPrime_a[2][2], hPrime_a[2][3], 0.0, 0.0, 0.0}};//concatenate hPrime_a with a 3x3 zeros matrix horizontally

  float tmp2[3][7] = {{hPrime_m[0][0], hPrime_m[0][1], hPrime_m[0][2], hPrime_m[0][3], 0.0, 0.0, 0.0},
                      {hPrime_m[1][0], hPrime_m[1][1], hPrime_m[1][2], hPrime_m[1][3], 0.0, 0.0, 0.0},
                      {hPrime_m[2][0], hPrime_m[2][1], hPrime_m[2][2], hPrime_m[2][3], 0.0, 0.0, 0.0}};//concatenate hPrime_m with a 3x3 zeros matrix horizontally

  for (int i = 0; i < 3; i++){
    for (int v = 0; v < 7; v++){
      C[i][v] = tmp1[i][v];
    }
  }
  for (int i = 3; i < 6; i++){
    for (int v = 0; v < 7; v++){
      C[i][v] = tmp2[i-3][v];
    }
  }//concatenate tmp1 and tmp2 vertically
  //HERE end the predictAccelMag function

  for (int i = 0; i < 3; i++){
    yHatBar[i] = accelBar[i];
  }
  for (int i = 3; i < 6; i++){
    yHatBar[i] = magBar[i-3];
  }//concatenate accelBar and magBar

  float inter1[7][7];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 7; z++){
        inter_result += A[i][z]*p[z][v];
      }
      inter1[i][v] = inter_result;
    }
  }//multiplication of A[7x7] with p[7x7]

  float A_trans[7][7];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      A_trans[i][v] = A[v][i];
    }
  }//tranposes 7x7 matrix A

  float inter2[7][7];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 7; z++){
        inter_result += inter1[i][z]*A_trans[z][v];
      }
      inter2[i][v] = inter_result;
    }
  }//multiplication of inter1[7x7] with A_trans[7x7]

  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      pBar[i][v] = inter2[i][v] + Q[i][v];
    }
  }//sum inter2 and Q
  //HERE ENDS THE PREDICT STEP
  //UPDATE
  float mezi1[6][7];
  for (int i = 0; i < 6; i++){
    for (int v = 0; v < 7; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 7; z++){
        inter_result += C[i][z]*pBar[z][v];
      }
      mezi1[i][v] = inter_result;
    }
  }//multiplication of C[6x7] with pBar[7x7]

  float C_trans[7][6];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 6; v++){
      C_trans[i][v] = C[v][i];
    }
  }//tranposes 6x7 matrix C

  float mezi2[6][6];
  for (int i = 0; i < 6; i++){
    for (int v = 0; v < 6; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 7; z++){
        inter_result += mezi1[i][z]*C_trans[z][v];
      }
      mezi1[i][v] = inter_result;
    }
  }//multiplication of mezi1[6x7] with C_trans[7x6]

  mtx_type mezi3[6][6];
  for (int i = 0; i < 6; i++){
    for (int v = 0; v < 6; v++){
      mezi3[i][v] = mezi2[i][v] + R[i][v];
    }
  }
  Matrix.Invert((mtx_type*)mezi3, 6);

  float mezi4[7][6];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 6; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 7; z++){
        inter_result += pBar[i][z]*C_trans[z][v];
      }
      mezi4[i][v] = inter_result;
    }
  }//multiplication of pBar[7x7] with C_trans[7x6]

  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 6; v++){
      float inter_result = 0.0;
      for(int z = 0; z < 6; z++){
        inter_result += mezi4[i][z]*mezi3[z][v];
      }
      K[i][v] = inter_result;
    }
  }//multiplication of mezi4[7x6] with mezi3[6x6]

  //HERE starts getMagVector (based on magnetometer readings)
  temp1[0][0] = xHat[0] * xHat[0] + xHat[1] * xHat[1] - xHat[2] * xHat[2] - xHat[3] * xHat[3];
  temp1[0][1] = 2 * (xHat[1] * xHat[2] - xHat[0] * xHat[3]);
  temp1[0][2] = 2 * (xHat[1] * xHat[3] + xHat[0] * xHat[2]);
  temp1[1][0] = 2 * (xHat[1] * xHat[2] + xHat[0] * xHat[3]);
  temp1[1][1] = xHat[0] * xHat[0] - xHat[1] * xHat[1] + xHat[2] * xHat[2] - xHat[3] * xHat[3];
  temp1[1][2] = 2 * (xHat[2] * xHat[3] - xHat[0] * xHat[1]);
  temp1[2][0] = 2 * (xHat[1] * xHat[3] - xHat[0] * xHat[2]);
  temp1[2][1] = 2 * (xHat[2] * xHat[3] + xHat[0] * xHat[1]);
  temp1[2][2] = xHat[0] * xHat[0] - xHat[1] * xHat[1] - xHat[2] * xHat[2] + xHat[3] * xHat[3];

  float magGauss_N[3];//3x1 matrix
  for (int i = 0; i < 3; i++){
    float inter_result = 0.0;
    for (int v = 0; v < 3; v++){
      inter_result += temp1[i][v]*magnetometer_cal[v];
    }
    magGauss_N[i] = inter_result;
  }//mulitplication of temp1[3x3] with magnetometer_cal[3x1]

  //magGauss_N[2] = 0.0;//no clue why we do this, just copying python code, originally this was uncommented!!!!!!
  float magGauss_N_mag = sqrt(magGauss_N[0]*magGauss_N[0] + magGauss_N[1]*magGauss_N[1] + magGauss_N[2]*magGauss_N[2]);
  for (int i = 0; i < 3; i++){
    magGauss_N[i] /= magGauss_N_mag;
  }

  float temp1_trans[3][3];
  for (int i = 0; i < 3; i++){
    for (int v = 0; v < 3; v++){
      temp1_trans[i][v] = temp1[v][i];
    }
  }//transpose of temp1

  float magGauss_B[3];
  for (int i = 0; i < 3; i++){
    float inter_result = 0.0;
    for (int v = 0; v < 3; v++){
      inter_result += temp1_trans[i][v]*magGauss_N[v];
    }
    magGauss_B[i] = inter_result;
  }//mulitplication of temp1_trans[3x3] with magGauss_N[3x1]
  //HERE end getMagVector
  //HERE starts getAccelVector
  float accelMag = sqrt(accel_cal[0]*accel_cal[0] + accel_cal[1]*accel_cal[1] + accel_cal[2]*accel_cal[2]);
  float accel_B[3];
  for (int i = 0; i < 3; i++){
    accel_B[i] = accel_cal[i]/accelMag;
  }
  //HERE end getAccelVector
  float measurement[6] = {accel_B[0]-yHatBar[0], accel_B[1]-yHatBar[1], accel_B[2]-yHatBar[2], magGauss_B[0]-yHatBar[3], magGauss_B[1]-yHatBar[4], magGauss_B[2]-yHatBar[5]};//6x1 matrix
  float mezi5[7];//7x1 matrix
  for (int i = 0; i < 7; i++){
    float inter_result = 0.0;
    for (int v = 0; v < 6; v++){
      inter_result += K[i][v]*measurement[v];
    }
    mezi5[i] = inter_result;
  }//multiplication of K[7x6] with measurement[6x1]

  for (int i = 0; i < 7; i++){
    xHat[i] = xHatBar[i] + mezi5[i];
  }//sum of xHatBar and mezi5

  float xHat_mag = sqrt(xHat[0]*xHat[0] + xHat[1]*xHat[1] + xHat[2]*xHat[2] + xHat[3]*xHat[3]);
  for (int i = 0; i < 4; i++){
    xHat[i] = xHat[i]/xHat_mag;
  }

  float mezi6[7][7];
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      float inter_result = 0.0;
      for (int z = 0; z < 6; z++){
        inter_result += K[i][z]*C[z][v];
      }
      mezi6[i][v] = inter_result;
    }
  }//multiplication of K[7x6] with C[6x7]

  float mezi7[7][7] = {{1-mezi6[0][0], -mezi6[0][1], -mezi6[0][2], -mezi6[0][3], -mezi6[0][4], -mezi6[0][5], -mezi6[0][6]},
                       {-mezi6[1][0], 1-mezi6[1][1], -mezi6[1][2], -mezi6[1][3], -mezi6[1][4], -mezi6[1][5], -mezi6[1][6]},
                       {-mezi6[2][0], -mezi6[2][1], 1-mezi6[2][2], -mezi6[2][3], -mezi6[2][4], -mezi6[2][5], -mezi6[2][6]},
                       {-mezi6[3][0], -mezi6[3][1], -mezi6[3][2], 1-mezi6[3][3], -mezi6[3][4], -mezi6[3][5], -mezi6[3][6]},
                       {-mezi6[4][0], -mezi6[4][1], -mezi6[4][2], -mezi6[4][3], 1-mezi6[4][4], -mezi6[4][5], -mezi6[4][6]},
                       {-mezi6[5][0], -mezi6[5][1], -mezi6[5][2], -mezi6[5][3], -mezi6[5][4], 1-mezi6[5][5], -mezi6[5][6]},
                       {-mezi6[6][0], -mezi6[6][1], -mezi6[6][2], -mezi6[6][3], -mezi6[6][4], -mezi6[6][5], 1-mezi6[6][6]}};//7x7 identity matrix - mezi6
  
  for (int i = 0; i < 7; i++){
    for (int v = 0; v < 7; v++){
      float inter_result = 0.0;
      for (int z = 0; z < 7; z++){
        inter_result += mezi7[i][z]*pBar[z][v];
      }
      p[i][v] = inter_result;
    }
  }//multiplication of mezi7[7x7] with pBar[7x7]

  float m[3][3] = {{xHat[0] * xHat[0] + xHat[1] * xHat[1] - xHat[2] * xHat[2] - xHat[3] * xHat[3], 2 * (xHat[1] * xHat[2] - xHat[0] * xHat[3]), 2 * (xHat[1] * xHat[3] + xHat[0] * xHat[2])},
                   {2 * (xHat[1] * xHat[2] + xHat[0] * xHat[3]), xHat[0] * xHat[0] - xHat[1] * xHat[1] + xHat[2] * xHat[2] - xHat[3] * xHat[3], 2 * (xHat[2] * xHat[3] - xHat[0] * xHat[1])},
                   {2 * (xHat[1] * xHat[3] - xHat[0] * xHat[2]), 2 * (xHat[2] * xHat[3] + xHat[0] * xHat[1]), xHat[0] * xHat[0] - xHat[1] * xHat[1] - xHat[2] * xHat[2] + xHat[3] * xHat[3]}};

  if (-m[2][0] > 0.99999){
    yaw = 0;
    pitch = 3.14/2;
    roll = atan2f(m[0][1], m[0][2]);
  }
  else if(-m[2][0] < -0.99999){
    yaw = 0;
    pitch = -3.14/2;
    roll = atan2f(-m[0][1], -m[0][2]);
  }
  else{
    yaw = atan2f(m[1][0], m[0][0]);
    pitch = asinf(-m[2][0]);
    roll = atan2f(m[2][1], m[2][2]);
  }
  yaw = yaw*180.0/3.14;
  pitch = pitch*180.0/3.14;
  roll = roll*180.0/3.14;
  // roll = atan2f(xHat[0]*xHat[1] + xHat[2]*xHat[3], 0.5 - xHat[1]*xHat[1] - xHat[2]*xHat[2])*180.0/3.14;
  // pitch = asinf(-2.0*(xHat[1]*xHat[3] - xHat[0]*xHat[2]))*180.0/3.14;
  // yaw = atan2f(xHat[1]*xHat[2] + xHat[0]*xHat[3], 0.5 - xHat[2]*xHat[2] - xHat[3]*xHat[3])*180.0/3.14;
}

// float concenate(float A, float B, int axis){
//   if (axis = 1){
//     int rows = sizeof(A)/sizeof(A[0]);
//     int collumn_A = sizeof(A[0])/sizeof(A[0][0]);
//     int collumn_B = sizeof(B[0])/sizeof(B[0][0]);
//     int collumns = collumn_A + int collumn_B;
//     float pica[rows][collumns];
//     for (int i = 0; i < rows; i++){
//       for (int p = 0; p < collumn_A; p++){
//         pica[i][p] = A[i][p];
//       }
//       for (int y = collumn_A; y < collumns; y++){
//         pica[i][y] = B[i][y-collumn_A];
//       }
//     }
//       return pica;
//   }
//   else{
//     int collumns = sizeof(A[0])/sizeof(A[0][0]);
//     int rows_A = sizeof(A)/sizeof(A[0]);
//     int rows_B = sizeof(B)/sizeof(B[0]);
//     int rows = rows_A + rows_B);
//     float pica[rows][collumns];
//     for (int i = 0; i < collumns; i++){
//       for (int p = 0; p < rows_A; p++){
//         pica[p][i] = A[p][i];
//       }
//       for (int y = rows_A; y < rows; y++){
//         pica[y][i] = B[y-rows_A][i];
//       }
//     }
//       return pica;
//   }
// }this shouldve been a fucntion that would be used over and over but it does not seem to be possible to program in C++ as C++ does not allow for flexible memory usage!!! cant do nothing like python

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