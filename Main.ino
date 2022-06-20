
/* **********CU ROCKET ENGINEERING 2022***************
 *  
 *  AVIONICS DATA COLLECTION AND TRANSMISSION
 * ---------------------------------------------------
 * 
 * Features:
 * -Gyro orientation dead reckoning using Quaternion rotations
 * -Launch detection
 * -Continuous data logging to SD and real-time via RF
 * 
 * Sensor set used:
 * -MPU 9250 9DOF
 * -BME 280 Pressure/Alt./Humidity/Temp.
 * -Sparkfun UBLOX GPS breakout SAM M8Q
 * 
 * Accelerometer SHOULD be reasonably well calibrated prior to launch!
 * 
 * Notes:
 * - 0 Degree tilt reading implies close to horizontal flight
 * 
 * - This program will NOT function well for determining 
 *   orientation under very fast/erratic rotations exceeding 2000dps
 *   and should ONLY be used for rocket flight due to the nature of 
 *   the sensors used as well as the accuracy of 'dead-reckoning'.
 *   
 * - Prior to launch the program is in "launch mode 0", meaning that the
 *   orientation information is derived from the accelerometer and the gravity
 *   vector that it detects.
 *   
 *   Once the net g forces detected are above a defined threshold, the program
 *   switches to "launch mode 1" where dead-reckoning with the gyros is
 *   used to determine the new orientation every few microseconds. 
 *   These rotations are computed using quaternions to prevent the common errors
 *   of Euler angle orientation from interfering from true values. 
 *   (Ex. gimbal lock)
 *   Due to the issues of dead-reckoning, longer flights will unfortunately suffer
 *   from gyro drift error where the orientation drifts from true as a funtion of
 *   time. Flights lasting within 1 minute should have decent orientation info. for
 *   the launch portion of the flight.
 *   
 * - The sparkfun lsm9ds1 library in conjunction with the lsm9ds1 9dof stick
 *   showed pretty abysmal performance for dead reckoning using the gyros. 
 *   AVOID using this sensor if you are not using the MPU9250 or MPU 6050.
 * 
 * - Expect approximately 1 degree less accurate orientation per minute
 * 
 * {Accelerometer calibration results 4/13/2022}
 * 
 * [subtract bias, then multiply by scale]
 * 
 * For CURE Spaceport America Cup 2022
 * 
 * Written by: Filip Bronola
 */
 
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include "MPU9250.h"
//Use local MPU9250.h file ("MPU9250.h" as opposed to <MPU9250.h>)
//to ensure the correct parameters are set.
//Some MPU9250 parameters can only be set in the header file!
//Using the default library settings may yield poor readings.
//Example: +-16g Accelerometer range setting should be used, default settings are +-2g.

#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS

#define SEALEVELPRESSURE_HPA (1013.25)

#define chipSelect 10

#define I2Cclock 400000
#define I2Cport Wire1
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
Adafruit_BME280 bme;                     
SFE_UBLOX_GNSS myGNSS;

uint8_t bmeaddr = 0x77;

#define dataInt 50         //Interval between data recording (ms)

#define axscale 0.998185
#define ayscale 0.999094
#define azscale 0.986415

#define axbias 0.085618*2048
#define aybias 0.042423*2048
#define azbias 0.100746*2048

float gox, goy, goz;
float dA[3];                //Delta angle between timed intervals;

float rgx, rgy, rgz;
float accx, accy, accz;     //Acceleration directly recorded
float taccx, taccy, taccz;  //Acc. only due to wind/rocket thrust
float R, Rnew;              //Acceleration net, and acc. minus 1g

float angXZ, angYZ, angZZ;   //Tilt angles of each accelerometer axis.

//*NOTE* angle ZZ represents the true tilt of the rocket if the 
//Z axis of the accelerometer is parallel with the rocket tube length
//and the xy plane of the accelerometer is parallel with the cross section
//of the rocket.
  
unsigned int startT, t;
int launchMode = 0;

unsigned int curTime;
unsigned int tHour = 0;
unsigned int tMin = 0;
unsigned int tSec = 0;
unsigned int tMillis = 0;
unsigned int LT; //Launch time set after triggered

//Quaternion Setup
float qO[4] = {1,0,0,0}; //'Orientation' unit quaternion q[0] = w and q[1-3] = complex vector component
float qR[4];             //Rotation quaternion

String dataString = "";
char buf[512];
String fileName = "";
char fName[11];
//All data at will be stored on this String and saved to the SD card and transmitted
//dataString order:
//{GPS TIME, T - Launch, etc..} 

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);
  SD.begin(chipSelect);
  
  bme.begin(bmeaddr,&Wire2);
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X8, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X8,
                    Adafruit_BME280::STANDBY_MS_0_5 );
  
  Wire1.begin();
  Wire2.begin();
  Wire.begin();
  Wire.setClock(400000);

  myGNSS.begin(Wire,0x42,250,false);
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(10); //
  myGNSS.setAutoPVT(true);
  
  myGNSS.getPVT();

  while(myGNSS.getSIV() < 5);// Wait for satellites to determine file name! 

  myGNSS.getPVT();
  
  fileName += String(myGNSS.getMonth());
  fileName += "_";
  fileName += String(myGNSS.getDay());
  fileName += "_";
  fileName += String(myGNSS.getYear());
  fileName += "_";
  fileName += String(myGNSS.getHour());
  fileName += "_";
  fileName += String(myGNSS.getMinute());
  fileName += ".txt";

  Serial.println(fileName);
  
  myIMU.initMPU9250();
  
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();
  gyroCal();

  startT = millis();
  
}

void loop() {
  if(launchMode == 0){
    //launchMode = 1; //Only uncomment to override pre-launch for testing stuff!
    
    while(launchMode == 0){
      myIMU.readAccelData(myIMU.accelCount);
      
      accx = ((float)myIMU.accelCount[0] + axbias ) * myIMU.aRes * axscale - 0.1;
      accy = ((float)myIMU.accelCount[1] + aybias ) * myIMU.aRes * ayscale - 0.05;
      accz = ((float)myIMU.accelCount[2] + azbias ) * myIMU.aRes * azscale - 0.1;
  
      R = sqrt(sq(accx)+sq(accy)+sq(accz));
      
      if((R > 0.985) && (R < 1.015)){
        //Initialize quaternion orientation with accelerometer
        //This is done to get initial orientation until just prior to launch
        //Will only read data within a few percent of 1g.
        qInit(accx,accy,accz);
        }
      else if(R >= 3){
        launchMode = 1;
        LT = millis();
        }
      else{
        ; //Don't record any data!
        }
      if(millis() - startT >= dataInt){
        LT = millis();
        dataGrab();
        startT = millis();
        }
      //launchMode = 1; //ANOTHER OVERRIDE, DO NOT UNCOMMENT UNLESS NEEDED
      }
    
    startT = millis();
    
    myIMU.readGyroData(myIMU.gyroCount);
    rgx = (float)myIMU.gyroCount[0] * myIMU.gRes - gox;
    rgy = (float)myIMU.gyroCount[1] * myIMU.gRes - goy;
    rgz = (float)myIMU.gyroCount[2] * myIMU.gRes - goz;
    t = micros();
  }
  float rgx, rgy, rgz;
  float dT;
  
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){

    myIMU.readAccelData(myIMU.accelCount);
    
    accx = ((float)myIMU.accelCount[0] + axbias ) * myIMU.aRes * axscale - 0.1;
    accy = ((float)myIMU.accelCount[1] + aybias ) * myIMU.aRes * ayscale - 0.05;
    accz = ((float)myIMU.accelCount[2] + azbias ) * myIMU.aRes * azscale - 0.1;
  
    R = sqrt(sq(accx)+sq(accy)+sq(accz));

    myIMU.readGyroData(myIMU.gyroCount);
    
    rgx = (float)myIMU.gyroCount[0] * myIMU.gRes - gox;
    rgy = (float)myIMU.gyroCount[1] * myIMU.gRes - goy;
    rgz = (float)myIMU.gyroCount[2] * myIMU.gRes - goz;

    dT = ((float)(micros()-t))/1000000;
    t = micros();
  
    dA[0] = (rgx)*dT;
    dA[1] = (rgy)*dT;
    dA[2] = (rgz)*dT;
    
    qRotate(-dA[0], dA[2], -dA[1]);
  
  if(millis() - startT >= dataInt){
    tiltCalc();
    trueAccel();
    dataGrab();
    startT = millis();
    }
  }
}

void dataGrab(){
  timeCall();
  if(myGNSS.getPVT());

  dataString = "";
  ///*
  dataString += String(myGNSS.getHour());
  dataString += ":";
  dataString += String(myGNSS.getMinute());
  dataString += ":";
  dataString += String(myGNSS.getSecond());
  dataString += ".";
  int ms = myGNSS.getMillisecond();
    if (ms < 100)
      dataString += "0";
    if (ms < 10)
      dataString += "0";
    dataString += String(ms);
  dataString += " ";
  dataString += String(tHour);
  dataString += ":";
  dataString += String(tMin);
  dataString += ":";
  dataString += String(tSec);
  dataString += ".";
  dataString += String(tMillis);
  dataString += " ";
  dataString += String(degrees(angZZ),1);
  dataString += "° ";
  dataString += "Without G:";
  dataString += " ";
  dataString += String(taccx,4);
  dataString += " ";
  dataString += String(taccy,4);
  dataString += " ";
  dataString += String(taccz,4);
  dataString += " ";
  dataString += "R-1G: ";
  dataString += String(Rnew,4);
  dataString += " ";
  dataString += "With G:";
  dataString += " ";
  dataString += String(accx,4);
  dataString += " ";
  dataString += String(accy,4);
  dataString += " ";
  dataString += String(accz,4);
  dataString += " ";
  dataString += "R: ";
  dataString += String(R,4);
  dataString += " ";
  dataString += "Speed: ";
  dataString += String((float)myGNSS.getGroundSpeed()*0.00223694,2);
  dataString += " [mph] Heading: ";
  dataString += String((float)myGNSS.getHeading()*pow(10,-5),2);
  dataString += "° Alt.GPS: ";
  dataString += String((float)myGNSS.getAltitudeMSL()/1000);
  dataString += " Alt.BARO: ";
  dataString += String(bme.readAltitude(SEALEVELPRESSURE_HPA),2);
  dataString += " LAT: ";
  dataString += String((float)myGNSS.getLatitude()*pow(10,-7),6);
  dataString += " LONG: ";
  dataString += String((float)myGNSS.getLongitude()*pow(10,-7),6);
  dataString += " SIV: ";
  dataString += String(myGNSS.getSIV());

  File dataFile = SD.open(fileName.c_str(), FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
  startT = millis();
  
  //Serial.println(dataString); //Uncomment to print over USB serial
  dataString.toCharArray(buf, 512);
  Serial1.write(buf, 512);
  
  //printQuat(); //Uncomment to view quaternion over serial usb
}

void tiltCalc(){
  //Quick example:
  //Calculate tilt from global Z-axis... Represent axis w/ unit vector: [0,0,1] or [0,0,0,1] quaternion notation
  //Multiply the inverse of the current quaternion by the vector
  //Using quaternion multiplication sequence P' = q*P*(q^(-1))
  
  float qX[4] = {0,1,0,0}; //Quaternion representing ground axis X
  float qY[4] = {0,0,1,0}; //Quaternion representing ground axis Y
  float qZ[4] = {0,0,0,1}; //Quaternion representing vertical axis Z

  float qXRi[4]; //Intermediate quaternion essentially = q*P
  float qYRi[4]; //^^^^^^^^^^^^
  float qZRi[4]; //^^^^^^^^^^^^

  float qI[4]; //Conjugate of orientation quaternion

  qI[0] = qO[0];
  for(int i = 1; i < 4; i++){
    qI[i] = -1*qO[i];
  }
  
  float qXR; //Quaternion representing angle from axis X to Z
  float qYR; //Quaternion representing angle from axis Y to Z
  float qZR; //Quaternion representing angle from axis Z to Z
  
  qZRi[0] = qO[0]*qZ[0] - qO[1]*qZ[1] - qO[2]*qZ[2] - qO[3]*qZ[3];
  qZRi[1] = qO[1]*qZ[0] + qO[0]*qZ[1] + qO[2]*qZ[3] - qO[3]*qZ[2];
  qZRi[2] = qO[0]*qZ[2] - qO[1]*qZ[3] + qO[2]*qZ[0] + qO[3]*qZ[1];
  qZRi[3] = qO[0]*qZ[3] + qO[1]*qZ[2] - qO[2]*qZ[1] + qO[3]*qZ[0];

  //Leaving the lines of code below commented to show how the other components are left out.
  //Notice that we only need angle information from vertical axis Z aka the gravity vector
  //in order to remove the gravity component from each accelerometer axis, so we record the
  //resulting 'k' component or array index 3 if there were an array storing the entire quat. product.
  
  //qZR[0] = qZRi[0]*qI[0] - qZRi[1]*qI[1] - qZRi[2]*qI[2] - qZRi[3]*qI[3];
  //qZR[1] = qZRi[1]*qI[0] + qZRi[0]*qI[1] + qZRi[2]*qI[3] - qZRi[3]*qI[2];
  //qZR[2] = qZRi[0]*qI[2] - qZRi[1]*qI[3] + qZRi[2]*qI[0] + qZRi[3]*qI[1];
  qZR/*[3]*/ = qZRi[0]*qI[3] + qZRi[1]*qI[2] - qZRi[2]*qI[1] + qZRi[3]*qI[0];

  qXRi[0] = qO[0]*qX[0] - qO[1]*qX[1] - qO[2]*qX[2] - qO[3]*qX[3];
  qXRi[1] = qO[1]*qX[0] + qO[0]*qX[1] + qO[2]*qX[3] - qO[3]*qX[2];
  qXRi[2] = qO[0]*qX[2] - qO[1]*qX[3] + qO[2]*qX[0] + qO[3]*qX[1];
  qXRi[3] = qO[0]*qX[3] + qO[1]*qX[2] - qO[2]*qX[1] + qO[3]*qX[0];

  qXR = qXRi[0]*qI[3] + qXRi[1]*qI[2] - qXRi[2]*qI[1] + qXRi[3]*qI[0];

  qYRi[0] = qO[0]*qY[0] - qO[1]*qY[1] - qO[2]*qY[2] - qO[3]*qY[3];
  qYRi[1] = qO[1]*qY[0] + qO[0]*qY[1] + qO[2]*qY[3] - qO[3]*qY[2];
  qYRi[2] = qO[0]*qY[2] - qO[1]*qY[3] + qO[2]*qY[0] + qO[3]*qY[1];
  qYRi[3] = qO[0]*qY[3] + qO[1]*qY[2] - qO[2]*qY[1] + qO[3]*qY[0];

  qYR = qYRi[0]*qI[3] + qYRi[1]*qI[2] - qYRi[2]*qI[1] + qYRi[3]*qI[0];
  
  angXZ = asin(qXR);
  angYZ = asin(qYR);
  angZZ = asin(qZR); //"tilt" angle
  
}

void trueAccel(){
  taccx = accx + sin(angXZ);
  taccy = accy + sin(angYZ);
  taccz = accz - sin(angZZ);

  Rnew = sqrt(sq(taccx) + sq(taccy) + sq(taccz));
}

void qInit(float accx, float accy, float accz){
  
  float pitch = 0, roll = 0, yaw = 0;
  
  pitch = atan2(-accx, sqrt(sq(accy)+sq(accz)));
  roll = atan2(accy,accz);
  //Yaw does not need to be calculated as the device will be constantly
  //yawing or 'rolling' around the local Z-axis. 
  //Yaw is initiated with 0.

  angZZ = ((3.14159)/2) - atan(sqrt((sq(tan(roll)))+sq(tan(pitch)))); 

  qGround(-degrees(roll),yaw,-degrees(pitch));
}

void qGround(float Ax, float Ay, float Az){
  float c1, c2, c3;
  float s1, s2, s3;
  float qMag = 0;       //Magnitude for normalization
  
  Ax = radians(Ax)/2;
  Ay = radians(Ay)/2;
  Az = radians(Az)/2;

  c1 = cos(Az); s1 = sin(Az);
  c2 = cos(Ay); s2 = sin(Ay);
  c3 = cos(Ax); s3 = sin(Ax);

  qO[0] = c1*c2*c3 - s1*s2*s3;
  qO[1] = s1*s2*c3 + c1*c2*s3;
  qO[2] = s1*c2*c3 + c1*s2*s3;
  qO[3] = c1*s2*c3 - s1*c2*s3;

  //Quaternion Normalization
  for(int i = 0; i < 4; i++){
    qMag += sq(qO[i]);
  }
  qMag = sqrt(qMag);
  
  for(int i = 0; i < 4; i++){
    qO[i] = qO[i]/qMag; 
  }
  
}

void qRotate(float Ax, float Ay, float Az){
  float c1, c2, c3;
  float s1, s2, s3;
  float q[4];       //Quaternion multiplication product
  float qMag = 0;       //Magnitude for normalization
  
  Ax = radians(Ax)/2;
  Ay = radians(Ay)/2;
  Az = radians(Az)/2;

  c1 = cos(Az); s1 = sin(Az);
  c2 = cos(Ay); s2 = sin(Ay);
  c3 = cos(Ax); s3 = sin(Ax);

  qR[0] = c1*c2*c3 - s1*s2*s3;
  qR[1] = s1*s2*c3 + c1*c2*s3;
  qR[2] = s1*c2*c3 + c1*s2*s3;
  qR[3] = c1*s2*c3 - s1*c2*s3;

  q[0] = qO[0]*qR[0] - qO[1]*qR[1] - qO[2]*qR[2] - qO[3]*qR[3];
  q[1] = qO[1]*qR[0] + qO[0]*qR[1] + qO[2]*qR[3] - qO[3]*qR[2];
  q[2] = qO[0]*qR[2] - qO[1]*qR[3] + qO[2]*qR[0] + qO[3]*qR[1];
  q[3] = qO[0]*qR[3] + qO[1]*qR[2] - qO[2]*qR[1] + qO[3]*qR[0];

  //Quaternion Normalization
  for(int i = 0; i < 4; i++){
    qMag += sq(q[i]);
  }
  qMag = sqrt(qMag);
  
  for(int i = 0; i < 4; i++){
    q[i] = q[i]/qMag; 
    qO[i] = q[i];             //Normalized quaternion product is stored as orientation
  }
}

void printQuat(){
    Serial.print("Quaternion: ");
    Serial.print(qO[0],2);
    Serial.print(", ");
    Serial.print(qO[1],2);
    Serial.print(", ");
    Serial.print(qO[2],2);
    Serial.print(", ");
    Serial.println(qO[3],2);
}

void timeCall(){
  curTime = millis() - LT;
  tHour = curTime/(60*60000);
  curTime = curTime-tHour*(60*60000);
  tMin = curTime/(60000);
  curTime = curTime-tMin*(60000);
  tSec = curTime/(1000);
  curTime = curTime-tSec*(1000);
  tMillis = curTime;
}

void gyroCal(){
  unsigned int t = millis();
  float gOffset = 0;
  for(int i = 0; i < 1000; i++){
    myIMU.readGyroData(myIMU.gyroCount);
    gOffset += (float)myIMU.gyroCount[0] * myIMU.gRes;
    while(millis() - t < 1);
    t = millis();
  }
 gox = gOffset/1000;
 gOffset = 0;
  for(int i = 0; i < 1000; i++){
    myIMU.readGyroData(myIMU.gyroCount);
    gOffset += (float)myIMU.gyroCount[1] * myIMU.gRes;
    while(millis() - t < 1);
    t = millis();
  }
 goy = gOffset/1000;
 gOffset = 0;
  for(int i = 0; i < 1000; i++){
    myIMU.readGyroData(myIMU.gyroCount);
    gOffset += (float)myIMU.gyroCount[2] * myIMU.gRes;
    while(millis() - t < 1);
    t = millis();
  }
 goz = gOffset/1000;
 }
