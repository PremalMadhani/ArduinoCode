class MyInsKalman {
  public:
    float angle; // Estimated angle
    float bias;  // Estimated gyro bias
    float rate;  // Unbiased rate

    float P[2][2]; // Error covariance matrix

    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias;  // Process noise variance for the gyro bias
    float R_angle; // Measurement noise variance

    MyInsKalman() {
      // Default Tuning Parameters
      Q_angle = 0.001f;
      Q_bias  = 0.003f;
      R_angle = 0.03f;

      angle = 0.0f;
      bias  = 0.0f;

      P[0][0] = 0.0f; P[0][1] = 0.0f;
      P[1][0] = 0.0f; P[1][1] = 0.0f;
    }

    float predictAndUpdate(float newAngle, float newRate, float dt) {
      // --- PREDICT ---
      rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      // --- UPDATE ---
      float y = newAngle - angle;
      float S = P[0][0] + R_angle;

      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      angle += K[0] * y;
      bias  += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }

    void setAngle(float newAngle) { angle = newAngle; }
};