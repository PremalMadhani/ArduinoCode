#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;
 
float thetaG=0;
float phiG=0;
 
float theta;
float phi;
 
float thetaRad;
float phiRad;
 
float Xm;
float Ym;
float psi;
 
 
float dt;
unsigned long millisOld;
 
#define BNO055_SAMPLERATE_DELAY_MS (100)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();
 
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
millisOld=millis();
}

void loop() {
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mg = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mg);
imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

// Read the quaternion from the BNO055 hardware fusion engine
imu::Quaternion quat = myIMU.getQuat();

// Extract the 4 scalar components:
float qW = quat.w(); // Scalar / Real part
float qX = quat.x(); // i component (Vector X)
float qY = quat.y(); // j component (Vector Y)
float qZ = quat.z(); // k component (Vector Z)

float Qroll_rad  = atan2(2.0 * (qW * qX + qY * qZ), 1.0 - 2.0 * (qX * qX + qY * qY));
float Qpitch_rad = asin(2.0 * (qW * qY - qZ * qX));
float Qphi   = Qroll_rad * 180.0 / M_PI;  // Roll in degrees
float Qtheta = Qpitch_rad * 180.0 / M_PI; // Pitch in degrees

thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
phiM=atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
phiFnew=.95*phiFold+.05*phiM;
thetaFnew=0.95*thetaFold+.05*thetaM;
 
dt=(millis()-millisOld)/1000.;
millisOld=millis();
theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
phi=(phi+gyr.x()*dt)*.95+ phiM*.05;
thetaG=thetaG+gyr.y()*dt;
phiG=phiG+gyr.x()*dt;
 
phiRad=phi/360*(2*3.14);
thetaRad=theta/360*(2*3.14);
 
Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
 
psi=atan2(Ym,Xm)/(2*3.14)*360;

/*Serial.print("Calibration");
Serial.print(" Accel:");
Serial.print(accel);
Serial.print(",");
Serial.print("Gyro:");
Serial.print(gyro);
Serial.print(",");
Serial.print("Mag:");
Serial.print(mg);
Serial.print(",");
Serial.print("System:");
Serial.print(system);*/
/*Serial.print(" RawAccelX:"); 
Serial.print(acc.x()/9.8);
Serial.print(",");
Serial.print("RawAccelY:");
Serial.print(acc.y()/9.8);
Serial.print(",");
Serial.print("RawAccelZ:");
Serial.println(acc.z()/9.8);
Serial.print(",");*/
Serial.print("Quatheta:");
Serial.print(Qtheta);
Serial.print(",");
Serial.print("Quaphi:");
Serial.print(Qphi);
Serial.print(",");
Serial.print("RawAcceltheta:");
Serial.print(thetaM);
Serial.print(",");
Serial.print("RawAccelphi:");
Serial.print(phiM);
Serial.print(",");
Serial.print("FiltAcceltheta:");
Serial.print(thetaFnew);
Serial.print(",");
Serial.print("FiltAccelphi:");
Serial.print(phiFnew);
Serial.print(",");
/*Serial.print("RawGyrotheta:");
Serial.print(thetaG);
Serial.print(",");
Serial.print("RawGyrophi:");
Serial.print(phiG);
Serial.print(",");*/
Serial.print("Compfilttheta:");
Serial.print(theta);
Serial.print(",");
Serial.print("Compfiltphi:");
Serial.print(phi);
Serial.print(",");
Serial.print("Compfiltpsi:");
Serial.println(psi);
 
phiFold=phiFnew;
thetaFold=thetaFnew;
 
 
delay(BNO055_SAMPLERATE_DELAY_MS);
}
