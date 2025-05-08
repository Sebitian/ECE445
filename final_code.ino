#include <ESP32Servo.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include "ThingSpeak.h"
#include <math.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "pitches.h"

#define LED_PIN   7
#define SERVO_PIN 14
#define SDA_PIN     21   // your SDA pin
#define SCL_PIN     18   // your SCL pin
#define NOTE_E5  659
#define BUZZER_PIN  4

// FSM
enum LockState { LOCKED, UNLOCKED, ALARM };
LockState currentState = LOCKED;
LockState previousState = LOCKED; // Track previous state to detect changes
const float IMU_THRESHOLD = 13.0;

// MPU6050
Adafruit_MPU6050 mpu;
float temperatureC;

//==== WIFI ====
// const char* ssid       = "YOUR_SSID";       // Replace with your WiFi name
// const char* password   = "YOUR_PASSWORD";   // Replace with your WiFi password
const char* ssid       = "YOUR_WIFI_SSID";
const char* password   = "YOUR_WIFI_PASSWORD";

// === THINGSPEAK =============================================
unsigned long myChannelNumber = 1;
const char*    myWriteAPIKey  = "YOUR_THINGSPEAK_API_KEY";
const char*    serverName     = "http://api.thingspeak.com/update";

// SERVO 
Servo myservo;

// BLE ===============================================
// BLE Server callbacks (connection state)

// COMMANDS
void reset_alarm(); // type 'reset'
void play_rickroll(); // type 'rick'
void play_got_theme(); // type 'got'
void play_melody(); 
void send_to_thinkspeak();

static bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer)   override { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer)override { 
    deviceConnected = false; 
    pServer->startAdvertising(); 
  }
};

// BLE Characteristic write handler
class CharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rx = pChar->getValue();
    if (rx.equals("open")) {
      digitalWrite(LED_PIN, HIGH);
      myservo.attach(SERVO_PIN, 1000, 2000);
      for (int pos = 0; pos <= 180; pos += 2) {
        myservo.write(pos);
        delay(15);
      }
      myservo.detach();
      
      // Change state to UNLOCKED when opening via Bluetooth
      if (currentState != UNLOCKED) {
        previousState = currentState;
        currentState = UNLOCKED;
        // Send state change to ThingSpeak
        send_to_thinkspeak();
        Serial.println("STATE: Changed to UNLOCKED");
      }
    }
    else if (rx.equals("close")) {
      digitalWrite(LED_PIN, LOW);
      myservo.attach(SERVO_PIN, 1000, 2000);
      for (int pos = 180; pos >= 0; pos -= 2) {
        myservo.write(pos);
        delay(15);
      }
      myservo.detach();
      
      // Change state to LOCKED when closing via Bluetooth
      if (currentState != LOCKED) {
        previousState = currentState;
        currentState = LOCKED;
        // Send state change to ThingSpeak
        send_to_thinkspeak();
        Serial.println("STATE: Changed to LOCKED");
      }
    }
    else if (rx.equals("reset")) {
      // Reset the alarm state via Bluetooth
      reset_alarm();
    }
    else if (rx.equals("rick")) {
      // Play the Rick Roll melody when receiving "rick" command
      Serial.println("Rick Roll activated!");
      play_rickroll();
    }
    else if (rx.equals("got")) {
      // Play the Game of Thrones theme when receiving "got" command
      Serial.println("Game of Thrones theme activated!");
      play_got_theme();
    }
  }
};


//==== BUZZER ====
int melody[]       = { NOTE_E5, NOTE_E5, NOTE_E5 };
int noteDurations[] = { 8, 8, 4 };

// WIFI
void init_WiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(700);
  }
  // WiFi connected - solid LED for 3 seconds
  digitalWrite(LED_PIN, HIGH);
  delay(3000);
  digitalWrite(LED_PIN, LOW);
}

// MELODY
void play_melody() {
  // Calculate the actual number of notes in the active melody
  int size = sizeof(melody) / sizeof(int);

  for (int thisNote = 0; thisNote < size; thisNote++) {
    // Calculate note duration
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // Pause between notes
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    
    // Stop the tone
    noTone(BUZZER_PIN);
  }
}

// LED
void init_led() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

// BLE
void init_ble() {
  
  // Prepare PWM timers (don't attach yet)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // BLE setup
  BLEDevice::init("ESP32-S3");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService("12345678-1234-1234-1234-1234567890ab");
  BLECharacteristic* pChar = pService->createCharacteristic(
    "abcdefab-1234-1234-1234-abcdefabcdef",
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pChar->addDescriptor(new BLE2902());
  pChar->setValue("Ready");
  pChar->setCallbacks(new CharCallbacks());

  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(pService->getUUID());
  BLEDevice::getAdvertising()->start();
}


void init_MPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // MPU failure pattern: very fast blinking (10 times)
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
    delay(1000);
  }
  
  // MPU found - indicate with a double blink pattern
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
}

// THINGSPEAK
void send_to_thinkspeak() {
  // Short blink to indicate HTTP request is starting
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  http.addHeader("Content-type", "application/x-www-form-urlencoded");
      
  // Get sensor readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply temperature calibration offset (MPU6050 needs calibration)
  temperatureC = temp.temperature;  // Apply offset correction
  
  // Set the status field based on the current state
  int statusValue;
  switch (currentState) {
    case LOCKED:
      statusValue = 1;  // LOCKED state = 1
      break;
    case UNLOCKED:
      statusValue = 0;  // UNLOCKED state = 0
      break;
    case ALARM:
      statusValue = 2;  // ALARM state = 2
      break;
    default:
      statusValue = 3;  // Unknown state = 3 (should never happen)
      break;
  }

  // Get acceleration data
  float accel_x = a.acceleration.x;
  float accel_y = a.acceleration.y;
  float accel_z = a.acceleration.z;

  // Blink LED once if data is read successfully
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);

  String httpRequestData = String("api_key=") + myWriteAPIKey +
                                "&field1=" + String(temperatureC, 2) +  // 2 decimal places precision
                              "&field2=" + statusValue +              // State indicator (1=LOCKED, 0=UNLOCKED, 2=ALARM)
                              "&field3=" + String(accel_x, 3) +       // x-axis acceleration
                              "&field4=" + String(accel_y, 3) +       // y-axis acceleration
                              "&field5=" + String(accel_z, 3);        // z-axis acceleration

  int httpResponseCode = http.POST(httpRequestData);
  
  // Indicate HTTP response status through LED
  if (httpResponseCode > 0) {
    // Success - LED stays on for 1 second
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
  } else {
    // Error - Blink quickly 5 times
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  http.end();
}

void update_lock_state() {
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate acceleration magnitude
  float accelerationMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                                     a.acceleration.y * a.acceleration.y + 
                                     a.acceleration.z * a.acceleration.z);
  
  // Account for gravity (approx 9.8 m/s²)
  accelerationMagnitude = abs(accelerationMagnitude - 9.8);
  
  // Save previous state before any changes
  previousState = currentState;
  
  // Check state transitions
  switch (currentState) {
    case LOCKED:
      // If in LOCKED state and acceleration exceeds threshold, trigger ALARM
      if (accelerationMagnitude > IMU_THRESHOLD) {
        currentState = ALARM;
        Serial.println("STATE: LOCKED -> ALARM (Motion detected!)");
        
        // Send alert to ThingSpeak when state changes to ALARM
        send_to_thinkspeak();
      }
      break;
      
    case UNLOCKED:
      // In UNLOCKED state, we don't trigger alarm on motion
      // But we could add other transitions here if needed
      break;
      
    case ALARM:
      // Periodic ThingSpeak updates while in ALARM state
      static unsigned long lastThingSpeakUpdate = 0;
      unsigned long now = millis();
      
      // Send update every 15 seconds while in ALARM state
      if (now - lastThingSpeakUpdate >= 15000) {
        send_to_thinkspeak();
        lastThingSpeakUpdate = now;
        Serial.println("Sending periodic alarm update to ThingSpeak");
      }
      break;
  }
  
  // Always send update to ThingSpeak if state changed
  if (currentState != previousState) {
    // Send update to ThingSpeak for any state change
    // (we already sent for LOCKED->ALARM above, but ThingSpeak has rate limits so it's okay)
    send_to_thinkspeak();
    Serial.print("STATE: Changed from ");
    Serial.print(stateToString(previousState));
    Serial.print(" to ");
    Serial.println(stateToString(currentState));
  }
}

// Helper function to convert state enum to string for debugging
String stateToString(LockState state) {
  switch (state) {
    case LOCKED: return "LOCKED";
    case UNLOCKED: return "UNLOCKED";
    case ALARM: return "ALARM";
    default: return "UNKNOWN";
  }
}

// Handle actions based on current state
void handle_state_actions() {
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  static unsigned long alarmStartTime = 0;
  
  switch (currentState) {
    case LOCKED:
      // In locked state, LED stays off
      digitalWrite(LED_PIN, LOW);
      // Reset alarm variables when returning to LOCKED state
      alarmStartTime = 0; // Reset the alarm start time so it will work again when triggered
      break;
      
    case UNLOCKED:
      // In unlocked state, LED stays on
      digitalWrite(LED_PIN, HIGH);
      // Reset alarm variables
      alarmStartTime = 0;
      break;
      
    case ALARM:
      // If this is the first time in ALARM state, record the start time
      if (alarmStartTime == 0) {
        alarmStartTime = millis();
        // Initial alert: LED on and play sound immediately
        digitalWrite(LED_PIN, HIGH);
        play_melody();
        ledState = true;
        lastToggle = millis();
      }
      
      // Continue flashing LED regardless of time spent in ALARM state
      unsigned long now = millis();
      if (now - lastToggle > 500) {
        // Toggle LED state
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastToggle = now;
        
        // Only play melody when LED turns ON and within first 10 seconds
        if (ledState && (now - alarmStartTime < 10000)) {
          play_melody(); // Play sound when LED turns on
        }
      }
      break;
  }
}

// Modified function to reset the alarm state via BLE
void reset_alarm() {
  LockState oldState = currentState;
  currentState = LOCKED;
  digitalWrite(LED_PIN, LOW);
  
  // Close the servo to ensure it's in locked position
  myservo.attach(SERVO_PIN, 1000, 2000);
  for (int pos = 180; pos >= 0; pos -= 2) {
    myservo.write(pos);
    delay(15);
  }
  myservo.detach();
  
  // Always send update to ThingSpeak when resetting alarm state
  Serial.println("STATE: Reset to LOCKED from " + stateToString(oldState));
  send_to_thinkspeak();  // Ensure ThingSpeak is updated regardless of previous state
}

// Add the Rick Roll melody notes and durations
// This is a simplified version of the melody using notes from pitches.h
void play_rickroll() {
  // Notes for "Never Gonna Give You Up" intro/chorus (simplified)
  int rickNotes[] = {
    NOTE_D5, NOTE_E5, NOTE_A4, NOTE_E5, NOTE_FS5, NOTE_A5, NOTE_G5, NOTE_FS5,
    NOTE_D5, NOTE_E5, NOTE_A4, NOTE_A4, NOTE_FS5, NOTE_G5, NOTE_E5
  };
  
  // Note durations: 4 = quarter note, 8 = eighth note, etc.
  int rickDurations[] = {
    8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 4, 8, 8, 8, 4
  };
  
  // Play each note in the melody
  for (int thisNote = 0; thisNote < sizeof(rickNotes) / sizeof(int); thisNote++) {
    // Calculate note duration
    int noteDuration = 1000 / rickDurations[thisNote];
    tone(BUZZER_PIN, rickNotes[thisNote], noteDuration);
    
    // Pause between notes (30% extra seems to sound good)
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    
    // Stop the tone
    noTone(BUZZER_PIN);
  }
}

// Add Game of Thrones theme song
#define REST 0  // Add REST note definition to play pauses

void play_got_theme() {
  // Game of Thrones theme song
  int got_melody[] = {
    NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
    NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
    NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4,
    NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4,
    NOTE_G4, NOTE_C4,
    
    NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
    NOTE_D4,
    NOTE_F4, NOTE_AS3,
    NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3,
    NOTE_DS4, NOTE_D4, NOTE_C4,
    
    NOTE_G4, NOTE_C4,
    
    NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
    NOTE_D4,
    NOTE_F4, NOTE_AS3,
    NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3,
    NOTE_DS4, NOTE_D4, NOTE_C4,
    NOTE_G4, NOTE_C4,
    NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
    
    NOTE_D4,
    NOTE_F4, NOTE_AS3,
    NOTE_D4, NOTE_DS4, NOTE_D4, NOTE_AS3,
    NOTE_C4,
    NOTE_C5,
    NOTE_AS4,
    NOTE_C4,
    NOTE_G4,
    NOTE_DS4,
    NOTE_DS4, NOTE_F4,
    NOTE_G4
  };
  
  int got_durations[] = {
    8, 8, 16, 16, 8, 8, 16, 16,
    8, 8, 16, 16, 8, 8, 16, 16,
    8, 8, 16, 16, 8, 8, 16, 16,
    8, 8, 16, 16, 8, 8, 16, 16,
    4, 4,
    
    16, 16, 4, 4, 16, 16,
    1,
    4, 4,
    16, 16, 4, 4,
    16, 16, 1,
    
    4, 4,
    
    16, 16, 4, 4, 16, 16,
    1,
    4, 4,
    16, 16, 4, 4,
    16, 16, 1,
    4, 4,
    16, 16, 4, 4, 16, 16,
    
    2,
    4, 4,
    8, 8, 8, 8,
    1,
    2,
    2,
    2,
    2,
    2,
    4, 4,
    1
  };
  
  // Play the melody
  int size = sizeof(got_durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    // Calculate note duration
    int duration = 1000 / got_durations[note];
    
    if (got_melody[note] == REST) {
      // If it's a rest, just delay without tone
      delay(duration * 1.30);
    } else {
      // Play the note
      tone(BUZZER_PIN, got_melody[note], duration);
      
      // Pause between notes (30% extra seems to work well)
      int pauseBetweenNotes = duration * 1.30;
      delay(pauseBetweenNotes);
      
      // Stop the tone
      noTone(BUZZER_PIN);
    }
  }
}

// ——————————————————————————————
// Initialization
void setup() {
  Serial.begin(115200);
  init_led();
  init_WiFi();
  init_ble();
  play_melody();
  init_MPU6050();
  
  // Start in LOCKED state
  currentState = LOCKED;
  
  // Send initial state to ThingSpeak during bootup
  Serial.println("STATE: Initial LOCKED state at bootup");
  send_to_thinkspeak();
}

void loop() {
  // Update lock state based on sensor readings
  update_lock_state();
  
  // Perform actions based on current state
  handle_state_actions();
  
  // BLE handling happens in callbacks
  
  // Short delay between updates
  delay(100);
}
