#include <MadgwickAHRS.h>
#include <Wire.h>
#include <ESP8266WiFi.h> 
#include <MPU6050.h>

MPU6050 mpu;
int SCL_PIN=D1;
int SDA_PIN=D2;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values

int yaw = 0;
Madgwick filter;
unsigned long microsPerReading, microsPrevious,microsNow;
float accelScale, gyroScale;

void setup() 
{
  WiFi.forceSleepBegin();// turn off ESP8266 RF
  delay(1);
  Serial.begin(115200);
  filter.begin(25);
   microsPerReading = 600000 / 25;
  microsPrevious = micros();

  // Initialize MPU6050
  while(!mpu.beginSoftwareI2C(SCL_PIN,SDA_PIN,MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
/*  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }*/
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();
   microsNow = micros();
   if (microsNow - microsPrevious >= microsPerReading)
   {
  // Read normalized values
      Vector ng = mpu.readNormalizeGyro();
      Vector na = mpu.readNormalizeAccel();
      filter.updateIMU(ng.XAxis,ng.YAxis,ng.ZAxis,na.XAxis,na.YAxis,na.ZAxis);

    // print the heading, pitch and roll
    
   yaw = filter.getYaw();
   yaw= yaw-180;
   // Serial.print("yaw ");
    //Serial.println(yaw);
   
     Serial.print(" Yaw = ");
  Serial.println(yaw);
    
   
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

  // Read normalized values
   }


  // Calculate Pitch, Roll and Yaw
//  pitch = pitch + norm.YAxis * timeStep;
//  roll = roll + norm.XAxis * timeStep;
//  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
 


  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
