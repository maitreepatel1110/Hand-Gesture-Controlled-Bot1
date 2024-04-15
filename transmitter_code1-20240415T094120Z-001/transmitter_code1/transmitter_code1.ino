#include  <Wire.h>
const  int MPU_ADDRESS = 0x68; 

float AccX, AccY, AccZ;
float  GyroX, GyroY, GyroZ;

float accAngleX, accAngleY;
float roll, pitch, yaw;

float AccErrorX, AccErrorY, GyroErrorX,  GyroErrorY, GyroErrorZ;

float elapsedTime, currentTime, previousTime;

int  c = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();                      
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);                 
  Wire.write(0x00);                 
  Wire.endTransmission(true);     
}

void  MPU_read_accel_data()
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); 
  
  //For a range of +-2g, we need to divide the  raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 |  Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read())  / 16384.0; 
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI)      - (-0.40); 
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ,  2))) * 180 / PI) - (-3.75);
  
}

void MPU_read_gyro_data()
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); 

  // For a 250deg/s range we have to divide first  the raw value by 131.0, according to the datasheet
  GyroX = (Wire.read() <<  8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX = GyroX - (-0.68); 
  GyroY = GyroY - (-2.48); 
  GyroZ = GyroZ  - (-0.12);  
}

void calculate_IMU_error()
{
  
  while (c < 200)
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS,  6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY  = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 |  Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX  + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY  = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180  / PI));
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  
  c =  0;
  
  // Read gyro values 200 times
  while (c < 200)
  {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS,  6, true);
    
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY =  Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ +  (GyroZ / 131.0);
    
    c++;
    
  }
  
  //Divide the sum  by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY  = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void loop()
{
  
  MPU_read_accel_data();
  MPU_read_gyro_data();

  previousTime = currentTime;        
  currentTime = millis();           
  elapsedTime = (currentTime - previousTime) / 1000; 
// Complementary filter - combine acceleromter  and gyro angle values
  roll  = 0.92 * (roll  + (GyroX * elapsedTime)) + 0.08  * accAngleX;
  pitch = 0.92 * (pitch + (GyroY * elapsedTime)) + 0.08 * accAngleY;

if(pitch<-17){
      Serial.write('F');
    }else if(pitch>20){
      Serial.write('B');
    }else if(roll>30){
      Serial.write('R');
    }else if(roll<-30){
      Serial.write('L');
    }else {
      Serial.write('S');
}
}
