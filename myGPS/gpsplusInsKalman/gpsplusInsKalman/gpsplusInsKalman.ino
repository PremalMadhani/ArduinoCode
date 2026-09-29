#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

SoftwareSerial gpsSerial(2, 3);
TinyGPSPlus gps;
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// GPS Home Anchor
double homeLat = 0.0, homeLon = 0.0;
bool homeSet = false;

// 1D Kalman Filter Class for Position and Velocity
class Kalman1D {
  public:
    float x; // State: [Position]
    float v; // State: [Velocity]
    float P[2][2]; // Error Covariance Matrix

    // Process Noise (Q) and Measurement Noise (R) tuning parameters
    float varAcc; // Accel noise variance (m/s^2)^2
    float varGPS; // GPS noise variance (m)^2

    Kalman1D(float processNoiseAcc, float gpsNoiseMeters) {
      x = 0.0; v = 0.0;
      P[0][0] = 1.0; P[0][1] = 0.0;
      P[1][0] = 0.0; P[1][1] = 1.0;
      varAcc = processNoiseAcc;
      varGPS = gpsNoiseMeters * gpsNoiseMeters;
    }

    // 1. PREDICT STEP (Run at ~50Hz IMU rate)
    void predict(float accel, float dt) {
      // Kinematic update: x = x + v*dt + 0.5*a*dt^2, v = v + a*dt
      x += v * dt + 0.5 * accel * dt * dt;
      v += accel * dt;

      // Update Error Covariance Matrix P = F*P*F' + Q
      float dt2 = dt * dt;
      float dt3 = dt2 * dt;
      float dt4 = dt2 * dt2;

      // Process noise covariance matrix Q (Discrete Wiener Process Noise)
      float q00 = 0.25 * dt4 * varAcc;
      float q01 = 0.5  * dt3 * varAcc;
      float q11 = dt2  * varAcc;

      float p00 = P[0][0], p01 = P[0][1], p10 = P[1][0], p11 = P[1][1];

      P[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 + q00;
      P[0][1] = p01 + dt * p11 + q01;
      P[1][0] = p10 + dt * p11 + q01;
      P[1][1] = p11 + q11;
    }

    // 2. UPDATE STEP (Run at ~1Hz GPS rate when new location arrives)
    void updateGPS(float z_gps) {
      // Innovation / Residual: y = Measurement - Estimate
      float y = z_gps - x;

      // Innovation Covariance: S = H*P*H' + R  (H = [1, 0])
      float S = P[0][0] + varGPS;

      // Kalman Gain: K = P*H' / S
      float K0 = P[0][0] / S;
      float K1 = P[1][0] / S;

      // State Update
      x += K0 * y;
      v += K1 * y;

      // Covariance Update: P = (I - K*H)*P
      float p00 = P[0][0], p01 = P[0][1];
      P[0][0] -= K0 * p00;
      P[0][1] -= K0 * p01;
      P[1][0] -= K1 * p00;
      P[1][1] -= K1 * p01;
    }

    // Zero Velocity Update (ZUPT) to kill residual stationary drift
    void applyZUPT() {
      v *= 0.1; // Rapidly damp velocity when stationary
    }
};

// Instantiate Filters: Process Noise = 0.5 (m/s^2)^2, GPS Noise = 2.5 meters
Kalman1D kalmanX(0.5, 2.5);
Kalman1D kalmanY(0.5, 2.5);

unsigned long lastTime = 0;
float posX_GPS = 0.0, posY_GPS = 0.0;

// Integrated INS State (Meters and Meters/sec)
float posX_INS = 0.0, posY_INS = 0.0;
float velX_INS = 0.0, velY_INS = 0.0;

void latLonToMeters(double lat, double lon, float &x, float &y) {
  const double EARTH_RADIUS = 6371000.0;
  double dLat = (lat - homeLat) * M_PI / 180.0;
  double dLon = (lon - homeLon) * M_PI / 180.0;
  x = dLon * cos(homeLat * M_PI / 180.0) * EARTH_RADIUS;
  y = dLat * EARTH_RADIUS;
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  if (!myIMU.begin()) {
    while (1);
  }
  myIMU.setExtCrystalUse(true);
  lastTime = micros();
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (!homeSet && gps.location.isValid()) {
    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    homeSet = true;
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    imu::Vector<3> linAcc = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingRad = euler.x() * M_PI / 180.0;

    // Convert Body Accel to Earth (East, North) Frame
    float accEast  = linAcc.x() * cos(headingRad) + linAcc.y() * sin(headingRad);
    float accNorth = -linAcc.x() * sin(headingRad) + linAcc.y() * cos(headingRad);

    // Apply Noise Threshold / Deadband
    if (abs(accEast) < 0.15)  accEast = 0.0;
    if (abs(accNorth) < 0.15) accNorth = 0.0;

    // Dead-Reckoning Integration
    velX_INS += accEast * dt;
    velY_INS += accNorth * dt;
    posX_INS += velX_INS * dt;
    posY_INS += velY_INS * dt;

    // --- KALMAN PREDICT STEP ---
    kalmanX.predict(accEast, dt);
    kalmanY.predict(accNorth, dt);

    // --- ZUPT (Zero Velocity Update) Threshold ---
    if (accEast == 0.0)  kalmanX.applyZUPT();
    if (accNorth == 0.0) kalmanY.applyZUPT();

    // --- KALMAN UPDATE STEP (When GPS Data Updates) ---
    if (homeSet && gps.location.isUpdated()) {
      latLonToMeters(gps.location.lat(), gps.location.lng(), posX_GPS, posY_GPS);
      kalmanX.updateGPS(posX_GPS);
      kalmanY.updateGPS(posY_GPS);

      // Complementary Weighting: Trust INS short-term, blend in GPS long-term (90% INS / 10% GPS)
      posX_INS = (0.90 * posX_INS) + (0.10 * posX_GPS);
      posY_INS = (0.90 * posY_INS) + (0.10 * posY_GPS);
    }

    // Output to Serial Plotter
    Serial.print(F("HomeLat:")); Serial.print(homeLat); Serial.print(F(","));
    Serial.print(F("HomeLon:")); Serial.print(homeLon); Serial.print(F(","));
    Serial.print("X_GPS(m):");    Serial.print(posX_GPS); Serial.print(",");
    Serial.print("Y_GPS(m):");    Serial.print(posY_GPS); Serial.print(",");
    Serial.print("X_INS(m):");    Serial.print(posX_INS); Serial.print(",");
    Serial.print("Y_INS(m):");    Serial.print(posY_INS); Serial.print(",");
    Serial.print("X_Kalman(m):"); Serial.print(kalmanX.x); Serial.print(",");
    Serial.print("Y_Kalman(m):"); Serial.print(kalmanY.x); Serial.println(",");
  }
}
