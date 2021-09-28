#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define GALILEO_BOARD
#define LED_PIN 13 

bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
const int lm_en  = 6;
const int lm_in1 = 7;
const int lm_in2 = 8;
const int rm_en  = 5;
const int rm_in1 = 12;
const int rm_in2 = 2;
void dmpDataReady() {
    mpuInterrupt = true;
}

  
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        int TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(F("MPU6050 connection "));
    Serial.print(mpu.testConnection() ? F("successful") : F("failed"));

    // wait for ready
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/
    /*this snippet was to tell the system the mpu is aligned. since it 
     * is commented, only turn the arduino on, when mpu is aligned
     */
    if (mpu.testConnecion()) == true
    {
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();  
    }
    // load and configure the DMP
    

    // supply your own gyro offsets here, scaled for min sensitivity (use mpucalib.ino if you want the values again, i'll send the file)
    // very important***
    mpu.setXGyroOffset(221);
    mpu.setYGyroOffset(27);
    mpu.setZGyroOffset(22);
    mpu.setZAccelOffset(200); // 1688 factory default for my test chip
    mpu.setXAccelOffset(-1507);
    mpu.setYAccelOffset(669);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
   // Serial.begin(9600);
    pinMode(lm_en, OUTPUT);
    pinMode(lm_in1, OUTPUT);
    pinMode(lm_in2, OUTPUT);
    pinMode(rm_en, OUTPUT);
    pinMode(rm_in1, OUTPUT);
    pinMode(rm_in2, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

    #ifdef ARDUINO_BOARD
        while (!mpuInterrupt && fifoCount < packetSize) {
        }
    #endif

    #ifdef GALILEO_BOARD
        delay(10);
    #endif

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //forward();
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);//getting yaw values
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);//getting roll values (ignore)
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI); // getting pitch values (ignore)
            float dev = (ypr[0] * 180/M_PI);  // only use yaw values
            Serial.println("dev");
            Serial.println(dev);
            if (dev > 5) //condition for left, adjust buffer value depending on requirement
            {
              Serial.println("Adjusting left");
              adjustleft();
             // delay(500);
            }
            else if (dev < -5) //condition for right, adjust buffer value depending on requirement
            {
              Serial.println("Adjusting right");
              adjustright();
              //delay(500);
            }
            else
            {
              forward();
            }
        #endif

     
    }
}

void adjustright()
{
  digitalWrite(lm_in1, LOW);
  digitalWrite(lm_in2, HIGH);
  
  digitalWrite(rm_in1, LOW);
  digitalWrite(rm_in2, HIGH);
  //gradually increasing speed
//  for (int i=0; i<255; i+=5){
  analogWrite(lm_en, 200);
  analogWrite(rm_en, 0);
}

void adjustleft()
{
 digitalWrite(lm_in1, LOW);
  digitalWrite(lm_in2, HIGH);
  
  digitalWrite(rm_in1, LOW);
  digitalWrite(rm_in2, HIGH);
  //gradually increasing speed
//  for (int i=0; i<255; i+=5){
  analogWrite(lm_en, 0);
  analogWrite(rm_en, 200); 
}

void forward()
{
  digitalWrite(lm_in1, LOW);
  digitalWrite(lm_in2, HIGH);
  
  digitalWrite(rm_in1, LOW);
  digitalWrite(rm_in2, HIGH);
  //gradually increasing speed
//  for (int i=0; i<255; i+=5){
  analogWrite(lm_en, 200);
  analogWrite(rm_en, 200);
}
