#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>

Adafruit_LSM6DSO32 IMU;

int state = 0;
String c;
String temp;
String act;
String adress;
String value;
String nrOfBytes;
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(250);
  IMUSetup();
  Wire.begin();
  Serial.println("Enter 'r' for read or 'w' for write:");
}

void loop() {
  if(Serial.available() > 0){
    c = Serial.readString();
    c.remove(c.length()-1);
    switch (state){
      case 0:
        act = c;
        state++;
        Serial.println("Enter register adress:");
        break;
      case 1:
        adress = c;
        state++;
        Serial.println("Enter desired register value:");
        break;
      case 2:
        value = c;
        state++;
        Serial.println("Enter numbre of bytes that is expected to be read:");
        break;
      case 3:
        nrOfBytes = c;
        state = 0;
        if(act == "r"){
          Serial.println(readRegister(adress.toInt(), nrOfBytes.toInt()));//read from desired register
        }
        else{
          writeRegister(adress.toInt(), value.toInt());//write new value into desired regsiter
          delay(100);
          Serial.println(readRegister(adress.toInt(), nrOfBytes.toInt()));//and subsequently read from it to check it was succesfully written
        }
        Serial.println("Enter 'r' for read or 'w' for write:");
        break;
    }
  }
}

int readRegister(int registryAdress,int bytes){
  Wire.beginTransmission(0x6A);//start communiation with slave                                    
  Wire.write(registryAdress);//access desired register                                                    
  Wire.endTransmission();//end communication with slave                                            
  Wire.requestFrom(0x6A,bytes);//request number of bytes from slave starting from the accessed register                                         
  while(Wire.available() < bytes);//wait until all bytes are received                                        
  if(bytes == 1){
    return Wire.read();//read the next available byte
  }
  else{
    return (Wire.read()<<8|Wire.read());//read the next available bytes, shift it by 8 bits(another byte) and then join it with the next available byte through an OR operation
  }
}

void writeRegister(int registryAdress, int desiredValue){
  Wire.beginTransmission(0x6A);//start communiation with slave                                        
  Wire.write(registryAdress);//access desired register
  Wire.write(desiredValue);//set the new registry value                                                   
  Wire.endTransmission();//end communication with slave  
}

void IMUSetup(){
  IMU.begin_I2C();
  IMU.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  IMU.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  IMU.setAccelDataRate(LSM6DS_RATE_208_HZ);
  IMU.setGyroDataRate(LSM6DS_RATE_208_HZ);
}






