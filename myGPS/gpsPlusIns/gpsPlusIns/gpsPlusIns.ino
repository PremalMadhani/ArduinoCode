#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// GPS on Pins 2 (RX) and 3 (TX)
SoftwareSerial gpsSerial(2, 3);
TinyGPSPlus gps;
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// GPS Anchor Coordinates (Home Position)
double homeLat = 0.0;
double homeLon = 0.0;
bool homeSet = false;

// Integrated INS State (Meters and Meters/sec)
float posX_INS = 0.0, posY_INS = 0.0;
float velX_INS = 0.0, velY_INS = 0.0;

// GPS State (Meters in Earth Frame)
float posX_GPS = 0.0, posY_GPS = 0.0;

unsigned long lastTime = 0;

// Function to convert Lat/Lon to local planar X/Y (Meters)
void latLonToMeters(double lat, double lon, float &x, float &y) {
  const double EARTH_RADIUS = 6371000.0; // Meters
  double dLat = (lat - homeLat) * M_PI / 180.0;
  double dLon = (lon - homeLon) * M_PI / 180.0;
  
  x = dLon * cos(homeLat * M_PI / 180.0) * EARTH_RADIUS;
  y = dLat * EARTH_RADIUS;
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  if(!myIMU.begin()) {
    Serial.println(F("BNO055 check failed!"));
    while(1);
  }
  myIMU.setExtCrystalUse(true);
  lastTime = micros();
  Serial.println(F("Waiting for GPS Fix..."));
}

void loop() {
  // 1. Process Incoming GPS Sentences
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Set Home Anchor on first valid GPS Fix
  if (!homeSet && gps.location.isValid()) {
    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    homeSet = true;
    Serial.println(F("GPS Home Anchor Set!"));
  }

  // 3. High-Frequency INS Prediction Step (~50 Hz)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  if (dt >= 0.02) { // 20 ms loop
    lastTime = now;

    // Read Linear Acceleration (Gravity pre-removed by BNO055 hardware)
    imu::Vector<3> linAcc = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // Read Orientation Quaternions to rotate body acceleration into Earth frame
    imu::Quaternion q = myIMU.getQuat();

    // Simplify body-to-earth rotation for 2D horizontal plane
    // Convert Body Accel (X, Y) to Earth Accel (East, North)
    imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingRad = euler.x() * M_PI / 180.0; // Heading in Radians

    float accEast  = linAcc.x() * cos(headingRad) + linAcc.y() * sin(headingRad);
    float accNorth = -linAcc.x() * sin(headingRad) + linAcc.y() * cos(headingRad);

    // Dead-Reckoning Integration
    velX_INS += accEast * dt;
    velY_INS += accNorth * dt;
    posX_INS += velX_INS * dt;
    posY_INS += velY_INS * dt;

    // 4. Low-Frequency GPS Correction Step (Triggers when new GPS data arrives ~1 Hz)
    if (homeSet && gps.location.isUpdated()) {
      latLonToMeters(gps.location.lat(), gps.location.lng(), posX_GPS, posY_GPS);

      // Complementary Weighting: Trust INS short-term, blend in GPS long-term (90% INS / 10% GPS)
      posX_INS = (0.90 * posX_INS) + (0.10 * posX_GPS);
      posY_INS = (0.90 * posY_INS) + (0.10 * posY_GPS);

      // Zero out velocity drift on strong GPS updates
      velX_INS *= 0.8;
      velY_INS *= 0.8;
    }

    // Output Fused Local Coordinates (Meters from starting point)
    Serial.print(F("HomeLat:")); Serial.print(homeLat); Serial.print(F(","));
    Serial.print(F("HomeLon:")); Serial.print(homeLon); Serial.print(F(","));
    Serial.print(F("X_Fused(m):")); Serial.print(posX_INS); Serial.print(F(","));
    Serial.print(F("Y_Fused(m):")); Serial.print(posY_INS); Serial.print(F(","));
    Serial.print(F("X_GPS(m):"));   Serial.println(posX_GPS);
  }
}