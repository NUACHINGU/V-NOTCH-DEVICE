# V-NOTCH-DEVICE
# ESP32 V-Notch Flow Measurement System

This project implements a real-time flow rate monitoring system using an **ESP32**, an **ultrasonic sensor**, and a **V-notch weir**. The system measures water level via ultrasonic distance sensing and calculates flow rate using the V-notch formula. A Bluetooth interface allows user-configurable parameters, and an I2C LCD displays live data.

---

## 📌 Features

### 📏 Distance & Flow Measurement
- Uses **HC-SR04** ultrasonic sensor (via `NewPing`) to measure water level
- Calculates flow rate based on the **V-notch weir formula**
- Median filtering with configurable sample count for accurate distance reading

### ⚙️ Flow Rate Calculation
- Supports customizable:
  - **Sensor height**
  - **Notch bottom height**
  - **Notch angle** (default 90°)
  - **Discharge coefficient (Cd)**
  - **k constant** (offset to account for sensor or notch setup)
- Outputs flow rate in **m³/h** (cubic meters per hour)

### 🧠 Configurable Parameters via Bluetooth
- Enter notch bottom height (`NOTCH_BOTTOM_HEIGHT`) and `K_CONSTANT` via Bluetooth at startup
- Bluetooth device name: `ESP32_VNotch`

### 📟 Display
- I2C 16x2 LCD shows:
  - Water height above the notch
  - Calculated flow rate in m³/h

### 🔁 Serial & Bluetooth Output
- Sends real-time data over Serial and Bluetooth in CSV format:


---

## 🧰 Hardware Requirements

| Component           | Description                          |
|---------------------|--------------------------------------|
| ESP32               | Main controller                      |
| HC-SR04             | Ultrasonic distance sensor           |
| LCD 16x2 (I2C)      | Display for live values              |
| Bluetooth Serial    | Built-in on ESP32 (uses `BluetoothSerial`) |
| Power Supply        | 5V regulated or USB                  |

**Pin Connections:**

| Function         | ESP32 Pin |
|------------------|-----------|
| Ultrasonic TRIG  | 25        |
| Ultrasonic ECHO  | 33        |
| I2C SDA (LCD)    | Default   |
| I2C SCL (LCD)    | Default   |

---

## ⚙️ Setup & Usage

### 🔧 Library Requirements
Install the following libraries via Arduino Library Manager:
- `NewPing`
- `LiquidCrystal_PCF8574`
- `BluetoothSerial` (included with ESP32 board package)

### 📲 Bluetooth Configuration
1. Upload the code to your ESP32
2. Connect via a Bluetooth terminal app (e.g., Serial Bluetooth Terminal)
3. On boot, input:
 - `NOTCH_BOTTOM_HEIGHT` (in cm)
 - `K_CONSTANT` (calibration offset in meters)
