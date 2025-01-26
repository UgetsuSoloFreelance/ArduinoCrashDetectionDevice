#include <Wire.h>
#include <MPU6050.h>

MPU6050 accel;

// UPDATE DELAYS
#define ACCEL_UPDATE_INTERVAL 100
unsigned long lastAccelUpdate = 0;

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
  unsigned long currentTime = millis();

  // ACCELEROMETER CODE
  if (currentTime - lastAccelUpdate >= ACCEL_UPDATE_INTERVAL) {
    lastAccelUpdate = currentTime;  // Update timestamp
    // RETRIEVE ACCELEROMETER DATA
    processAccelerometer();
  }
}

void processAccelerometer() {
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

bool classifyAndConfirmCrash(float shake, float ax, float ay, float az, float prevVx, float prevVy, float prevVz, 
                             float currVx, float currVy, float currVz, unsigned long currentTime) {
    static unsigned long crashStartTime = 0;  // Stores when a crash-like event starts
    static bool crashOngoing = false;         // Tracks if a crash is happening

    bool impactDetected = shake > SHAKE_THRESHOLD;
    bool speedDroppedX = abs(prevVx - currVx) > SPEED_DROP_THRESHOLD;
    bool speedDroppedY = abs(prevVy - currVy) > SPEED_DROP_THRESHOLD;
    bool speedDroppedZ = abs(prevVz - currVz) > SPEED_DROP_THRESHOLD;
    bool isFreeFall = (abs(ax) < FREE_FALL_THRESHOLD) && (abs(ay) < FREE_FALL_THRESHOLD) && (abs(az) < FREE_FALL_THRESHOLD);
    bool isShakingViolently = shake > SHAKE_REPEAT_THRESHOLD;
    bool isSpinning = abs(ax) > SPIN_THRESHOLD || abs(ay) > SPIN_THRESHOLD;

    // If a crash-like event is detected, start timing it
    if (impactDetected && (speedDroppedX || speedDroppedY || speedDroppedZ)) {
        if (!crashOngoing) {  // Start timing the crash
            crashStartTime = currentTime;
            crashOngoing = true;
        }
    } 
    else if (isFreeFall && impactDetected) {
        if (!crashOngoing) {
            crashStartTime = currentTime;
            crashOngoing = true;
        }
    }
    else if (isShakingViolently) {
        if (!crashOngoing) {
            crashStartTime = currentTime;
            crashOngoing = true;
        }
    }
    else if (isSpinning && (speedDroppedX || speedDroppedY || speedDroppedZ)) {
        if (!crashOngoing) {
            crashStartTime = currentTime;
            crashOngoing = true;
        }
    }
    else {
        // No crash detected, reset timer
        crashOngoing = false;
        crashStartTime = 0;
    }

    // Confirm the crash if the condition lasted longer than CRASH_TIME_LIMIT
    if (crashOngoing && (currentTime - crashStartTime >= CRASH_TIME_LIMIT)) {
        crashOngoing = false;  // Reset after confirming
        return true;
    }

    return false;
}