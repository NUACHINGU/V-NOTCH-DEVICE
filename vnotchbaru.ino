#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>  // ESP32-compatible I2C LCD library
#include <NewPing.h>
#include <math.h>
#include "BluetoothSerial.h"  // ESP32 Bluetooth Library

// Pin Definitions
#define TRIG_PIN 25
#define ECHO_PIN 33
#define MAX_DISTANCE 200  
#define SPEED_OF_SOUND 0.0343 
#define DISTANCE_OFFSET -3.62  
#define NUM_SAMPLES 10  

// V-Notch Parameters
#define SENSOR_HEIGHT 20  // Distance from sensor to V-notch crest (cm)
#define NOTCH_ANGLE 90.0  // Notch angle in degrees
#define CD 0.6  // Discharge coefficient
#define K_CONSTANT_DEFAULT 0.02  // Default k value (adjustable via Bluetooth)

BluetoothSerial BT;  // Initialize Bluetooth Serial
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_PCF8574 lcd(0x27);  // LCD I2C Address

float NOTCH_BOTTOM_HEIGHT = 15;  // Distance from tank bottom to V-notch bottom (cm)
float K_CONSTANT = K_CONSTANT_DEFAULT;  // Adjustable k value

// Median Filtering for Distance Measurement
float getFilteredDistance() {
  float readings[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) {
    readings[i] = (sonar.ping() * SPEED_OF_SOUND) / 2;
    delay(50);
  }
  // Sorting for Median
  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = i + 1; j < NUM_SAMPLES; j++) {
      if (readings[i] > readings[j]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[NUM_SAMPLES / 2] + DISTANCE_OFFSET;
}

// Calculate Flow Rate using the new Formula
float calculateFlowRate(float H) {
  if (H <= 0) return 0.0;  // No flow if water is below the notch
  
  float thetaRad = (NOTCH_ANGLE * M_PI) / 180.0; // Convert degrees to radians
  float H_m = H / 100.0;  // Convert cm to meters
  float Q = 4.28 * CD * tan(thetaRad / 2.0) * pow((H_m + K_CONSTANT), 2.5);
  return Q;  // Flow rate in cubic meters per second (m^3/s)
}

void setup() {
  Serial.begin(115200);
  BT.begin("ESP32_VNotch");  // Set Bluetooth device name
  lcd.begin(16, 2);
  lcd.setBacklight(255);

  lcd.setCursor(0, 0);
  lcd.print("Enter Notch H:");
  Serial.println("Enter Notch Bottom Height (cm) via Bluetooth:");

  while (BT.available() == 0) {
    // Wait for user input via Bluetooth
  }
  NOTCH_BOTTOM_HEIGHT = BT.parseFloat();
  Serial.print("Notch Bottom Height set to: ");
  Serial.print(NOTCH_BOTTOM_HEIGHT);
  Serial.println(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Enter k:");
  Serial.println("Enter k value via Bluetooth:");

  while (BT.available() == 0) {
    // Wait for user input via Bluetooth
  }
  K_CONSTANT = BT.parseFloat();
  Serial.print("k set to: ");
  Serial.println(K_CONSTANT);

  lcd.clear();
}

void loop() {
  float D = getFilteredDistance(); // Distance from sensor to water surface (cm)
  if (D < 0) D = 0;
  
  float H = D - SENSOR_HEIGHT; // Water height above V-notch crest
  if (H < 0) H = 0;  // No negative height
  
  float H_total = NOTCH_BOTTOM_HEIGHT + H; // Total water height from bottom to surface
  
  float Q = calculateFlowRate(H); // Compute flow rate in m^3/s
  
  // Convert flow rate to m³/h
  float Q_m3h = Q * 3600;
  
  static float lastQ = -1;
  if (abs(Q_m3h - lastQ) > 0.001) {  // Update only if flow changes significantly
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("H: ");
    lcd.print(H);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    lcd.print("Q: ");
    lcd.print(Q_m3h, 3);
    lcd.print(" m3/h");

    Serial.print("H: ");
    Serial.print(H);
    Serial.print(" cm, H_total: ");
    Serial.print(H_total);
    Serial.print(" cm, Q: ");
    Serial.print(Q_m3h, 3);
    Serial.println(" m^3/h");

    // Send Data via Bluetooth
    BT.print(H);
    BT.print(",");
    BT.print(H_total);
    BT.print(",");
    BT.println(Q_m3h, 3);

    lastQ = Q_m3h;
  }
  delay(500);
}
