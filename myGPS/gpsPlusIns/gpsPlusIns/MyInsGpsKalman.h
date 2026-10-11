// 1D Kalman Filter Class for Position and Velocity
class MyInsGpsKalman {
  public:
    float x; // State: [Position]
    float v; // State: [Velocity]
    float P[2][2]; // Error Covariance Matrix

    // Process Noise (Q) and Measurement Noise (R) tuning parameters
    float varAcc; // Accel noise variance (m/s^2)^2
    float varGPS; // GPS noise variance (m)^2

    MyInsGpsKalman(float processNoiseAcc, float gpsNoiseMeters) {
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
      v *= 0.9; // damp velocity when acc is zero
    }
};