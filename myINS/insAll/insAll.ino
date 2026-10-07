#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
//#include <Kalman.h> // Include TKJ Electronics Kalman library
#include "MyInsKalman.h" // My Kalman

MyInsKalman kalmanRoll;
MyInsKalman kalmanPitch;

Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Initialize Kalman instances
//Kalman kalmanPitch;
//Kalman kalmanRoll;

// Variable definitions
float thetaM, phiM;            // Raw Accelerometer Pitch & Roll
float thetaLPF = 0, phiLPF = 0;// Low-Pass Filtered values
float thetaComp = 0, phiComp = 0; // Complementary Filter values
float thetaKalman = 0, phiKalman = 0; // Kalman Filter values
float linAccelX_filt = 0.0, linAccelY_filt = 0.0, linAccelZ_filt = 0.0;

unsigned long timer = 0;
int lockoutTimer = 0;
const int LOCKOUT_SAMPLES = 25; // 250 ms cooldown at 100 Hz

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
  float gyroBias = 0;
  thetaComp = 0.95 * (thetaComp + (gyr.y()+gyroBias) * dt) + 0.05 * thetaM;
  phiComp   = 0.95 * (phiComp   + (gyr.x()+gyroBias) * dt) + 0.05 * phiM;

  // --- FILTER 3: 2-State Kalman Filter ---
  //thetaKalman = kalmanPitch.getAngle(thetaM, (gyr.y()+gyroBias), dt);
  //phiKalman   = kalmanRoll.getAngle(phiM, gyr.x(), dt);

  // --- FILTER 4: My 2 state kalman filter for roll and pitch
  float phiKalman  = kalmanRoll.predictAndUpdate(phiM, gyr.x()+gyroBias, dt);
  float thetaKalman = kalmanPitch.predictAndUpdate(thetaM, gyr.y()+gyroBias, dt);

  // --- FILTER 5: BNO055 Onboard Hardware Fusion ---
  imu::Vector<3> bnoEuler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  float bnoRoll  = -bnoEuler.y();
  float bnoPitch = bnoEuler.z();

  // Convert Fused Angles to Radians for Trigonometry
  float thetaRad = thetaComp * DEG_TO_RAD;
  float phiRad   = phiComp   * DEG_TO_RAD;

  // Calculate Expected Gravity Components on Body Axes (m/s^2)
  float gx = -9.81 * sin(thetaRad);
  float gy =  9.81 * sin(phiRad) * cos(thetaRad);
  float gz =  9.81 * cos(phiRad) * cos(thetaRad);

  float gxlpf = -9.81 * sin(thetaLPF*DEG_TO_RAD);
  // linear acc from lpf
  float linAccelXlpf = (acc.x() - gxlpf)/9.81;

  // ISOLATE TRUE LINEAR ACCELERATION (Feed these to ML model)
  float linAccelX = (acc.x() - gx)/9.81; // Pure Left/Right dynamic acceleration
  float linAccelY = (acc.y() - gy)/9.81; // Pure Forward/Backward dynamic acceleration
  float linAccelZ = (acc.z() - gz)/9.81; // Pure Up/Down dynamic acceleration

  // Low-Pass Filter (EMA) to attenuate motor/hand vibration noise
  // (alpha = 0.30 gives sharp response with minimal ~30ms lag)
  linAccelX_filt = 0.70 * linAccelX_filt + 0.30 * linAccelX;
  linAccelY_filt = 0.70 * linAccelY_filt + 0.30 * linAccelY;
  linAccelZ_filt = 0.70 * linAccelZ_filt + 0.30 * linAccelZ;

  // 7. Apply Baseline Noise Deadband (Clamps resting sensor jitter to exactly 0.0)
  if (abs(linAccelX_filt) < 0.04) linAccelX_filt = 0.0;
  if (abs(linAccelY_filt) < 0.04) linAccelY_filt = 0.0;
  if (abs(linAccelZ_filt) < 0.06) linAccelZ_filt = 0.0; // Slightly higher threshold for Z

  // Refractory Lockout Handling for Classification Logic
  bool readyForNewGesture = true;
  if (lockoutTimer > 0) {
    lockoutTimer--;
    readyForNewGesture = false;
  }

  // MOTION DETECTION LOGIC USING PURE ACCELERATION ---
  if (readyForNewGesture) {
    if (linAccelX_filt > 0.15) {
      Serial.println("MOTION DETECTED: RIGHT (+X)");
      lockoutTimer = LOCKOUT_SAMPLES; // Lockout for 250 ms to absorb deceleration phase
    } 
    else if (linAccelX_filt < -0.15) {
      Serial.println("MOTION DETECTED: LEFT (-X)");
      lockoutTimer = LOCKOUT_SAMPLES;
    }
    else if (linAccelY_filt > 0.15) {
      Serial.println("MOTION DETECTED: FORWARD (+Y)");
      lockoutTimer = LOCKOUT_SAMPLES;
    }
    else if (linAccelY_filt < -0.15) {
      Serial.println("MOTION DETECTED: BACKWARD (-Y)");
      lockoutTimer = LOCKOUT_SAMPLES;
    }
    else if (linAccelZ_filt > 0.20) {
      Serial.println("MOTION DETECTED: UP (+Z)");
      lockoutTimer = LOCKOUT_SAMPLES;
    }
    else if (linAccelZ_filt < -0.20) {
      Serial.println("MOTION DETECTED: DOWN (-Z)");
      lockoutTimer = LOCKOUT_SAMPLES;
    }
  }

  // 4. Output values formatted for Arduino Serial Plotter
  // Format: "Label1:Val1,Label2:Val2,..."
  Serial.print(timer); Serial.print(",");
  /*Serial.print(thetaM);      Serial.print(",");
  Serial.print(thetaLPF);    Serial.print(",");
  Serial.print(thetaComp);   Serial.print(",");
  Serial.print(thetaKalman); Serial.print(",");*/
  //Serial.print(bnoPitch); Serial.print(",");
  Serial.print(phiM);        Serial.print(",");
  Serial.print(phiLPF);    Serial.print(",");
  Serial.print(phiComp);   Serial.print(",");
  Serial.print(phiKalman); Serial.print(",");
  //Serial.println(bnoRoll); Serial.println(",");

  // Serial output for debugging/plotting linear motion
  Serial.print(linAccelXlpf); Serial.print(",");
  Serial.print(linAccelX); Serial.print(",");
  Serial.print(linAccelY); Serial.print(",");
  Serial.print(linAccelZ); Serial.print(",");
  Serial.print(linAccelX_filt); Serial.print(",");
  Serial.print(linAccelY_filt); Serial.print(",");
  Serial.print(linAccelZ_filt); Serial.print(",");
  Serial.print(readyForNewGesture ? 1:0); Serial.println(",");

  /*Serial.print("RawAccelPitch:");     Serial.print(thetaM);      Serial.print(",");
  Serial.print("LPFPitch:");          Serial.print(thetaLPF);    Serial.print(",");
  Serial.print("ComplementaryPitch:");Serial.print(thetaComp);   Serial.print(",");
  Serial.print("KalmanPitch:");       Serial.print(thetaKalman); Serial.print(",");
  Serial.print("BNO055_FusedPitch:"); Serial.print(bnoPitch); Serial.println(",");

  Serial.print("RawAccelRoll:");     Serial.print(phiM);        Serial.print(",");
  Serial.print("LPFRoll:");          Serial.print(phiLPF);    Serial.print(",");
  Serial.print("ComplementaryRoll:");Serial.print(phiComp);   Serial.print(",");
  Serial.print("KalmanRoll:");       Serial.print(phiKalman); Serial.print(",");
  Serial.print("BNO055_FusedRoll:"); Serial.println(bnoRoll);*/

  delay(10); // Run loop at ~100 Hz
}
