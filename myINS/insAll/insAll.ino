#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Kalman.h> // Include TKJ Electronics Kalman library

Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Initialize Kalman instances
Kalman kalmanPitch;
Kalman kalmanRoll;

// Variable definitions
float thetaM, phiM;            // Raw Accelerometer Pitch & Roll
float thetaLPF = 0, phiLPF = 0;// Low-Pass Filtered values
float thetaComp = 0, phiComp = 0; // Complementary Filter values
float thetaKalman = 0, phiKalman = 0; // Kalman Filter values

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  /*if(!myIMU.begin()) {
    Serial.println("BNO055 not detected!");
    while(1);
  }*/
  myIMU.begin();
  delay(1000);
  int8_t temp=myIMU.getTemp();
  myIMU.setExtCrystalUse(true);

  // Read initial accel angles to seed filters
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiM=atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;

  kalmanPitch.setAngle(thetaM);
  kalmanRoll.setAngle(phiM);

  timer = micros();
}

void loop() {
  // 1. Calculate delta time (dt)
  float dt = (micros() - timer) / 1000000.0;
  timer = micros();

  // 2. Get Raw Accelerometer & Gyroscope Vectors
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // deg/s

  // 3. Compute raw accelerometer pitch and roll
  //thetaM = -atan2(acc.x(), acc.z()) * 180.0 / M_PI;
  //phiM   =  atan2(acc.y(), acc.z()) * 180.0 / M_PI;
  thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiM=atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;

  // --- FILTER 1: Low-Pass Filter (LPF) ---
  thetaLPF = 0.90 * thetaLPF + 0.10 * thetaM;
  phiLPF   = 0.90 * phiLPF   + 0.10 * phiM;

  // --- FILTER 2: Complementary Filter ---
  thetaComp = 0.95 * (thetaComp + (gyr.y()+5) * dt) + 0.05 * thetaM;
  phiComp   = 0.95 * (phiComp   + gyr.x() * dt) + 0.05 * phiM;

  // --- FILTER 3: 2-State Kalman Filter ---
  thetaKalman = kalmanPitch.getAngle(thetaM, (gyr.y()+5), dt);
  phiKalman   = kalmanRoll.getAngle(phiM, gyr.x(), dt);

  // --- FILTER 4: BNO055 Onboard Hardware Fusion ---
  imu::Vector<3> bnoEuler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  float bnoRoll  = -bnoEuler.y();
  float bnoPitch = bnoEuler.z();

  // 4. Output values formatted for Arduino Serial Plotter
  // Format: "Label1:Val1,Label2:Val2,..."
  Serial.print("RawAccelPitch:");     Serial.print(thetaM);      Serial.print(",");
  Serial.print("LPFPitch:");          Serial.print(thetaLPF);    Serial.print(",");
  Serial.print("ComplementaryPitch:");Serial.print(thetaComp);   Serial.print(",");
  Serial.print("KalmanPitch:");       Serial.print(thetaKalman); Serial.print(",");
  Serial.print("BNO055_FusedPitch:"); Serial.print(bnoPitch); Serial.println(",");

  /*Serial.print("RawAccelRoll:");     Serial.print(phiM);        Serial.print(",");
  Serial.print("LPFRoll:");          Serial.print(phiLPF);    Serial.print(",");
  Serial.print("ComplementaryRoll:");Serial.print(phiComp);   Serial.print(",");
  Serial.print("KalmanRoll:");       Serial.print(phiKalman); Serial.print(",");
  Serial.print("BNO055_FusedRoll:"); Serial.println(bnoRoll);*/

  delay(10); // Run loop at ~100 Hz
}
