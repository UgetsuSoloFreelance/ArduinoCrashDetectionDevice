#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// PINS AND COMPONENTS
MPU6050 accel;
#define IRSensor 5
#define gpsSerial Serial
Adafruit_GPS GPS_SERIAL(&GPSSerial);
SoftwareSerial SIM900A(10, 11);

// motor crash variables
float velocityX = 0, velocityY = 0, velocityZ = 0;  // Speed in all 3 axes (m/s)
unsigned long prevTime = 0;
unsigned long lastImpactTime = 0;
bool isCrashConfirmed = false;
static unsigned long crashStartTime = 0;  // Stores when a crash-like event starts
static bool crashOngoing = false;         // Tracks if a crash is happening
unsigned long lastAccelUpdate = 0;

// SMS variables
int totalNumbers = 2;
int currentIndex = 0;
unsigned long lastSendTime = 0;
bool isSending = false;

// GPS VARIABLES
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
float latitude = 0.0, longitude = 0.0; // Variables to store coordinates
float recordedLat = 0.0, recordedLong = 0.0; // Variables to store recorded coordinates

/*
  THE FOLLOWING VARIABLES CAN BE ADJUSTED ACCORDING TO LIKING.
*/

// Intervals
#define ACCEL_UPDATE_INTERVAL 100
#define MESSAGE_INTERVAL 5000

// Motor Crash
const float SHAKE_THRESHOLD = 2.5;  // High impact threshold (g)
const float SPEED_DROP_THRESHOLD = 2.0;  // Speed drop threshold (m/s)
const float FREE_FALL_THRESHOLD = 0.3;  // Near-zero acceleration threshold (g) during free fall
const float SHAKE_REPEAT_THRESHOLD = 3.0;  // Violent shaking detection
const float SPIN_THRESHOLD = 1.5;  // Lateral acceleration for motor spinning out (g)
const unsigned long CRASH_TIME_LIMIT = 1000;  // Time (ms) to confirm crash (1 second)

// SMS
String PHONE_NUMBERS[] = {"+639936647951", "+639944344112"};

void setup() {
  Wire.begin();
  SIM900A.begin(9600);
  accel.initialize();

  if (!accel.testConnection()) {
    while (1);
  }

  prevTime = millis();  // Initialize time tracking
  lastImpactTime = 0;  // Initialize last impact time
  pinMode(IRSensor, INPUT);

  GPS_SERIAL.begin(9600);
  GPS_SERIAL.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_SERIAL.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop() {
  unsigned long currentTime = millis();

  bool isHelmetWorn = digitalRead(IRSensor) == 1;

  // ACCELEROMETER CODE
  if ((currentTime - lastAccelUpdate >= ACCEL_UPDATE_INTERVAL) && isHelmetWorn) {
    lastAccelUpdate = currentTime;  // Update timestamp
    // RETRIEVE ACCELEROMETER DATA
    processAccelerometer();
  }


  // MANAGE SMS
  if (SIM900A.available()>0) // check for pending messages to be received.
    SIM900A.read(); // disregard received messages.
  sendMessages(); // check for pending messages to be sent.

  // MANAGE GPS
  if (gps.newNMEAreceived()) {
    if (gps.parse(gps.lastNMEA())) {  // Parse valid GPS data
      latitude = gps.latitudeDegrees;
      longitude = gps.longitudeDegrees;
    }
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

  // process crash alert.
  if (isCrashConfirmed) {
    processCrashAlert();
    isCrashConfirmed = false;
  }
}

void processCrashAlert() {
  // TRIGGER SEND MESSAGE.
  recordedLat = latitude;
  recordedLong = longitude;
  currentIndex = 0; // Start from the first phone number
  isSending = true; // Enable message sending process
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
        crashStartTime = 0;
        return true;
    }

    return false;
}

void sendMessages() {
  if (isSending && currentIndex < totalNumbers) {
        if (millis() - lastSendTime >= MESSAGE_INTERVAL) {
            sendMessage(PHONE_NUMBERS[currentIndex]); // Send to current number
            lastSendTime = millis(); // Update timestamp
            currentIndex++; // Move to next number
        }
    } else {
        isSending = false; // Stop when all numbers are sent to
    }
}

void sendMessage(String recipient) {
  SIM900A.println("AT+CMGF=1"); // Set GSM module to text mode
  delay(100); // Short delay to allow command processing

  SIM900A.print("AT+CMGS=\"");
  SIM900A.print(recipient);
  SIM900A.println("\"");
  delay(100);

  SIM900A.println(createMessage()); // Message content
  delay(100);

  SIM900A.write(26); // CTRL+Z to send
  delay(100);
}

String createMessage() {
  String mapsLink = "https://www.google.com/maps/place/" + String(recordedLat, 6) + "," + String(recordedLong, 6);
  String message = "EMERGENCY: Motor crash detected.\nLocation: " + mapsLink;
  return message;
}
