#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "MyInsGpsKalman.h"

SoftwareSerial gpsSerial(2, 3);
TinyGPSPlus gps;
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// GPS Home Anchor
double homeLat = 0.0, homeLon = 0.0;
bool homeSet = false;

// Instantiate Filters: Process Noise = 0.5 (m/s^2)^2, GPS Noise = 2.5 meters
MyInsGpsKalman kalmanX(0.5, 2.5);
MyInsGpsKalman kalmanY(0.5, 2.5);

unsigned long lastTime = 0;
float posX_GPS = 0.0, posY_GPS = 0.0;

// Integrated INS State (Meters and Meters/sec)
float posX_INS = 0.0, posY_INS = 0.0;
float velX_INS = 0.0, velY_INS = 0.0;
float posX_Comp = 0.0, posY_Comp = 0.0;
float velX_Comp = 0.0, velY_Comp = 0.0;

// GPS Blackout Variables
bool simBlackout = false;
unsigned long blackoutStartTime = 0;
const unsigned long BLACKOUT_DURATION_MS = 10000; // 10-second outage

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

  // TRIGGER BLACKOUT VIA SERIAL COMMAND ('b' key) ---
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      simBlackout = true;
      blackoutStartTime = millis();
    }
  }

  // --- AUTOMATICALLY END BLACKOUT AFTER DURATION ---
  if (simBlackout && (millis() - blackoutStartTime >= BLACKOUT_DURATION_MS)) {
    simBlackout = false;
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    // Get bias compensated acc measurements which are compensated for gravity
    imu::Vector<3> linAcc = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Get Euler angle measurements based on bias compensated accelerometer and gyro meas
    // from BNO055 
    imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);

    /*float headingRad = euler.x() * M_PI / 180.0;
    // Convert Body Accel to Earth (East, North) Frame
    float accEast  = linAcc.x() * cos(headingRad) + linAcc.y() * sin(headingRad);
    float accNorth = -linAcc.x() * sin(headingRad) + linAcc.y() * cos(headingRad);*/

    float r = euler.z() * M_PI / 180.0; // Roll (radians)
    float p = euler.y() * M_PI / 180.0; // Pitch (radians)
    float y = euler.x() * M_PI / 180.0; // Yaw / Heading (radians)

    // Full 3D transformation to Earth ENU Frame (more accurate than just using heading)
    float accEast  = linAcc.x() * (cos(y)*cos(p)) 
                  + linAcc.y() * (cos(y)*sin(p)*sin(r) - sin(y)*cos(r)) 
                  + linAcc.z() * (cos(y)*sin(p)*cos(r) + sin(y)*sin(r));

    float accNorth = linAcc.x() * (sin(y)*cos(p)) 
                  + linAcc.y() * (sin(y)*sin(p)*sin(r) + cos(y)*cos(r)) 
                  + linAcc.z() * (sin(y)*sin(p)*cos(r) - cos(y)*sin(r));

    // Apply Noise Threshold / Deadband
    if (abs(accEast) < 0.05)  accEast = 0.0;
    if (abs(accNorth) < 0.05) accNorth = 0.0;

    // Dead-Reckoning Integration
    velX_INS += accEast * dt;
    velY_INS += accNorth * dt;
    posX_INS += velX_INS * dt;
    posY_INS += velY_INS * dt;

    velX_Comp += accEast * dt;
    velY_Comp += accNorth * dt;
    posX_Comp += velX_Comp * dt;
    posY_Comp += velY_Comp * dt;

    // damp complementary filter velocity when acc is zero
    if (accEast == 0.0)  velX_Comp *= 0.9;
    if (accNorth == 0.0) velY_Comp *= 0.9;

    // My experimental Kalman predict step
    kalmanX.predict(accEast, dt);
    kalmanY.predict(accNorth, dt);

    // --- ZUPT (Zero Velocity Update) Threshold ---
    if (accEast == 0.0)  kalmanX.applyZUPT();
    if (accNorth == 0.0) kalmanY.applyZUPT();

    // Update the filters (Complementary and experimental Kalman) with GPS meas and when no GPS blackout forced
    if (homeSet && gps.location.isUpdated() && !simBlackout) {
      latLonToMeters(gps.location.lat(), gps.location.lng(), posX_GPS, posY_GPS);

      // Extract GPS velocity components (m/s)
      float gpsSpeed = gps.speed.mps();
      float gpsHeadingRad = gps.course.deg() * M_PI / 180.0;
      float velX_GPS = gpsSpeed * sin(gpsHeadingRad);
      float velY_GPS = gpsSpeed * cos(gpsHeadingRad);

      // Complementary Weighting of position estimates: Trust INS short-term, blend in GPS long-term (90% INS / 10% GPS)
      posX_Comp = (0.90 * posX_Comp) + (0.10 * posX_GPS);
      posY_Comp = (0.90 * posY_Comp) + (0.10 * posY_GPS);

      // Complementary Weighting of velocity estimates (80% Accel / 20% GPS)
      velX_Comp = (0.80 * velX_Comp) + (0.20 * velX_GPS);
      velY_Comp = (0.80 * velY_Comp) + (0.20 * velY_GPS);

      // My experimental Kalman update stage for each of the two position dimensions, X and Y
      kalmanX.updateGPS(posX_GPS);
      kalmanY.updateGPS(posY_GPS);
    }

    // Output to Serial Plotter
    //Serial.print(F("HomeLat:")); Serial.print(homeLat); Serial.print(F(","));
    //Serial.print(F("HomeLon:")); Serial.print(homeLon); Serial.print(F(","));
    Serial.print(posX_GPS); Serial.print(",");
    Serial.print(posY_GPS); Serial.print(",");
    Serial.print(posX_INS); Serial.print(",");
    Serial.print(posY_INS); Serial.print(",");
    Serial.print(posX_Comp); Serial.print(",");
    Serial.print(posY_Comp); Serial.print(",");
    Serial.print(kalmanX.x); Serial.print(",");
    Serial.print(kalmanY.x); Serial.print(",");
    Serial.println(simBlackout ? 1 : 0);
  }
}
