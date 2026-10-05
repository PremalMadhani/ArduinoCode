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

  // --- TRIGGER BLACKOUT VIA SERIAL COMMAND ('b' key) ---
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

    imu::Vector<3> linAcc = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingRad = euler.x() * M_PI / 180.0;

    // Convert Body Accel to Earth (East, North) Frame
    float accEast  = linAcc.x() * cos(headingRad) + linAcc.y() * sin(headingRad);
    float accNorth = -linAcc.x() * sin(headingRad) + linAcc.y() * cos(headingRad);

    // Apply Noise Threshold / Deadband
    if (abs(accEast) < 0.05)  accEast = 0.0;
    if (abs(accNorth) < 0.05) accNorth = 0.0;

    // Dead-Reckoning Integration
    velX_INS += accEast * dt;
    velY_INS += accNorth * dt;
    posX_INS += velX_INS * dt;
    posY_INS += velY_INS * dt;
    //posX_Comp += velX_INS * dt;
    //posY_Comp += velY_INS * dt;
    velX_Comp += accEast * dt;
    velY_Comp += accNorth * dt;
    posX_Comp += velX_Comp * dt;
    posY_Comp += velY_Comp * dt;

    // --- KALMAN PREDICT STEP ---
    kalmanX.predict(accEast, dt);
    kalmanY.predict(accNorth, dt);

    // --- ZUPT (Zero Velocity Update) Threshold ---
    if (accEast == 0.0)  kalmanX.applyZUPT();
    if (accNorth == 0.0) kalmanY.applyZUPT();

    if (accEast == 0.0)  velX_Comp *= 0.1;
    if (accNorth == 0.0) velY_Comp *= 0.1;

    // --- KALMAN UPDATE STEP (When GPS Data Updates) ---
    if (homeSet && gps.location.isUpdated() && !simBlackout) {
      latLonToMeters(gps.location.lat(), gps.location.lng(), posX_GPS, posY_GPS);
      kalmanX.updateGPS(posX_GPS);
      kalmanY.updateGPS(posY_GPS);

      // Extract GPS velocity components (m/s)
      float gpsSpeed = gps.speed.mps();
      float gpsHeadingRad = gps.course.deg() * M_PI / 180.0;
      float velX_GPS = gpsSpeed * sin(gpsHeadingRad);
      float velY_GPS = gpsSpeed * cos(gpsHeadingRad);

      // Complementary Weighting: Trust INS short-term, blend in GPS long-term (90% INS / 10% GPS)
      posX_Comp = (0.90 * posX_Comp) + (0.10 * posX_GPS);
      posY_Comp = (0.90 * posY_Comp) + (0.10 * posY_GPS);

      // --- VELOCITY COMPLEMENTARY FILTER (80% Accel / 20% GPS Speed) ---
      velX_Comp = (0.80 * velX_Comp) + (0.20 * velX_GPS);
      velY_Comp = (0.80 * velY_Comp) + (0.20 * velY_GPS);
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
