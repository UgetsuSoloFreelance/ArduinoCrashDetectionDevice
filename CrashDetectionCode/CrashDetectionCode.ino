#include <Wire.h>
#include <MPU6050.h>

MPU6050 accel;

float velocityX = 0, velocityY = 0, velocityZ = 0;  // Speed in all 3 axes (m/s)
unsigned long prevTime = 0;
unsigned long lastImpactTime = 0;
bool isCrashConfirmed = false;

// Thresholds for motor crash classification
const float SHAKE_THRESHOLD = 2.5;  // High impact threshold (g)
const float SPEED_DROP_THRESHOLD = 2.0;  // Speed drop threshold (m/s)
const float FREE_FALL_THRESHOLD = 0.3;  // Near-zero acceleration threshold (g) during free fall
const float SHAKE_REPEAT_THRESHOLD = 3.0;  // Violent shaking detection
const float SPIN_THRESHOLD = 1.5;  // Lateral acceleration for motor spinning out (g)
const unsigned long CRASH_TIME_LIMIT = 1000;  // Time (ms) to confirm crash (1 second)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  accel.initialize();
  if (!accel.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  prevTime = millis();  // Initialize time tracking
  lastImpactTime = 0;  // Initialize last impact time
}

void loop() {
  // RETRIEVE ACCELEROMETER DATA
  int16_t ax, ay, az;
  accel.getAcceleration(&ax, &ay, &az);

  // Convert to acceleration in g
  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;

  // Get time difference in seconds
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // USE ACCELEROMETER DATA TO DETECT SHAKE INTENSITY
  float shake_intensity = measureShake(accel_x, accel_y, accel_z);

  // Store previous speed before updating
  float prevVelocityX = velocityX;
  float prevVelocityY = velocityY;
  float prevVelocityZ = velocityZ;

  // USE ACCELEROMETER DATA TO DETECT SPEED
  measureSpeed(accel_x, accel_y, accel_z, deltaTime);

  // USE SPEED AND SHAKE INTENCITY TO DETECT AND CONFIRM CRASH
  isCrashConfirmed = classifyAndConfirmCrash(shake_intensity, accel_x, accel_y, accel_z, prevVelocityX, prevVelocityY, prevVelocityZ, velocityX, velocityY, velocityZ, currentTime);

  // Print results
  if (isCrashConfirmed) {
    Serial.println("🚨 MOTOR CRASH CONFIRMED!");
  } else {
    Serial.println("No crash detected.");
  }

  delay(100);  // Adjust sample rate
}

// Function to calculate shake intensity (considering all axes)
float measureShake(float ax, float ay, float az) {
    float magnitude = sqrt(ax * ax + ay * ay + az * az);
    float baseline = 1.0;  // 1g (Earth's gravity)
    return magnitude - baseline;
}

// Function to calculate speed in all three axes
void measureSpeed(float ax, float ay, float az, float dt) {
  float g_to_m_s2 = 9.81;  // Convert g to m/s²

  // Remove gravity (assume gravity is in Z direction)
  float accel_x = ax * g_to_m_s2;
  float accel_y = ay * g_to_m_s2;
  float accel_z = az * g_to_m_s2;

  // Apply a simple threshold to filter out noise
  if (abs(accel_x) < 0.2) accel_x = 0;
  if (abs(accel_y) < 0.2) accel_y = 0;
  if (abs(accel_z) < 0.2) accel_z = 0;

  // Integrate acceleration to get velocity
  velocityX += accel_x * dt;
  velocityY += accel_y * dt;
  velocityZ += accel_z * dt;

  // Apply friction/damping to simulate real-world slow-down
  velocityX *= 0.98;
  velocityY *= 0.98;
  velocityZ *= 0.98;
}

// Function to classify the type of motor crash and confirm it
bool classifyAndConfirmCrash(float shake, float ax, float ay, float az, float prevVx, float prevVy, float prevVz, float currVx, float currVy, float currVz, unsigned long currentTime) {
  bool impactDetected = shake > SHAKE_THRESHOLD;
  bool speedDroppedX = abs(prevVx - currVx) > SPEED_DROP_THRESHOLD;
  bool speedDroppedY = abs(prevVy - currVy) > SPEED_DROP_THRESHOLD;
  bool speedDroppedZ = abs(prevVz - currVz) > SPEED_DROP_THRESHOLD;
  bool isFreeFall = (abs(ax) < FREE_FALL_THRESHOLD) && (abs(ay) < FREE_FALL_THRESHOLD) && (abs(az) < FREE_FALL_THRESHOLD);
  bool isShakingViolently = shake > SHAKE_REPEAT_THRESHOLD;
  bool isSpinning = abs(ax) > SPIN_THRESHOLD || abs(ay) > SPIN_THRESHOLD;

  // Confirm crash conditions based on various axes and behaviors
  if (impactDetected && (speedDroppedX || speedDroppedY || speedDroppedZ)) {
    lastImpactTime = currentTime;
    return true; // Crash confirmed by impact and speed drop
  } else if (!impactDetected && (speedDroppedX || speedDroppedY || speedDroppedZ) && (currentTime - lastImpactTime) < CRASH_TIME_LIMIT) {
    lastImpactTime = currentTime;
    return true; // Motor stopping abruptly
  } else if (isFreeFall && impactDetected) {
    lastImpactTime = currentTime;
    return true; // Free fall followed by impact
  } else if (isShakingViolently && (currentTime - lastImpactTime) < CRASH_TIME_LIMIT) {
    lastImpactTime = currentTime;
    return true; // Sustained shaking detected
  } else if (isSpinning && (speedDroppedX || speedDroppedY || speedDroppedZ)) {
    lastImpactTime = currentTime;
    return true; // Spinning out of control detected
  }

  return false; // No confirmed crash
}
