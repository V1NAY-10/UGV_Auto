// ---------- ESP32 Motor Bridge for the UGV ----------
// Drives 4 non-encoded DC motors through two L298N boards (skid steer).
//
// Link to the Raspberry Pi: wired GPIO UART, 115200 baud, 3.3 V logic on both sides
//     Pi GPIO14 / TXD (pin 8)   ->  ESP32 GPIO21 (RX)
//     Pi GPIO15 / RXD (pin 10)  <-  ESP32 GPIO22 (TX)
//     Pi GND (pin 6)            ->  ESP32 GND   (common ground is required)
//
// Command sources:
//  1. Raspberry Pi over the UART above (ROS 2 esp32_bridge.py node)
//       line format: "<left_pwm>,<right_pwm>\n", each in [-255, 255]
//       e.g. "150,150" forward, "-120,120" spin left, "0,0" stop
//     The same lines are also accepted on the USB port (Serial Monitor) for bench tests.
//  2. Bluetooth teleop (same keys as before, device "ESP32_Robot")
//       w/s/a/d move, x stop, l/h low/high speed
//     While a Bluetooth motion key is active, serial commands are ignored.
//     Send 'x' to stop and hand control back to the Pi.
//
// Safety: if no command arrives for SERIAL_TIMEOUT ms, motors stop.
// Motor pin mapping is unchanged from the original teleop firmware.

#include "BluetoothSerial.h"

// ===== Pi UART pins (GPIO16/17, the default UART2 pins, are used by motor M4) =====
#define PI_RX 21
#define PI_TX 22
#define PI_BAUD 115200

// ===== Motor Pins =====
#define M1_IN1 14
#define M1_IN2 27
#define M1_EN 26

#define M2_IN1 25
#define M2_IN2 33
#define M2_EN 32

#define M3_IN1 19
#define M3_IN2 18
#define M3_EN 5

#define M4_IN1 17
#define M4_IN2 16
#define M4_EN 4

// Left side = M1 + M4, right side = M2 + M3
// If one motor spins the wrong way, flip its entry here instead of rewiring.
const bool INVERT[4] = {false, false, false, false};  // M1, M2, M3, M4

const int IN1_PINS[4] = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
const int IN2_PINS[4] = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};
const int EN_PINS[4]  = {M1_EN, M2_EN, M3_EN, M4_EN};

// ===== PWM =====
// Lower PWM frequency gives the L298N more torque at low duty cycles
const int PWM_FREQ = 1000;
const int PWM_RES = 8;

// ===== Speed Settings (Bluetooth teleop) =====
const int LOW_SPEED = 120;
const int HIGH_SPEED = 220;
int speedPWM = LOW_SPEED;

// ===== Timing =====
const unsigned long SERIAL_TIMEOUT = 500;  // ms
unsigned long lastSerialCmd = 0;

BluetoothSerial BT;
char btCmd = 'x';
bool btActive = false;

String piLine = "";
String usbLine = "";

// ---------- Low level ----------
void pwmAttach(int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(EN_PINS[ch], PWM_FREQ, PWM_RES);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(EN_PINS[ch], ch);
#endif
}

void pwmWrite(int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(EN_PINS[ch], duty);
#else
  ledcWrite(ch, duty);
#endif
}

// Signed PWM for one motor: >0 forward, <0 backward, 0 coast
void setMotor(int m, int pwm) {
  if (INVERT[m]) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(IN1_PINS[m], HIGH); digitalWrite(IN2_PINS[m], LOW);
  } else if (pwm < 0) {
    digitalWrite(IN1_PINS[m], LOW);  digitalWrite(IN2_PINS[m], HIGH);
  } else {
    digitalWrite(IN1_PINS[m], LOW);  digitalWrite(IN2_PINS[m], LOW);
  }
  pwmWrite(m, abs(pwm));
}

void drive(int left, int right) {
  setMotor(0, left);   // M1
  setMotor(3, left);   // M4
  setMotor(1, right);  // M2
  setMotor(2, right);  // M3
}

void stopAll() { drive(0, 0); }

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);                              // USB: debug + bench tests
  Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX, PI_TX);  // Raspberry Pi
  BT.begin("ESP32_Robot");

  for (int m = 0; m < 4; m++) {
    pinMode(IN1_PINS[m], OUTPUT);
    pinMode(IN2_PINS[m], OUTPUT);
    pwmAttach(m);
  }
  stopAll();
  Serial.println("UGV motor bridge ready");
}

// ---------- Bluetooth teleop ----------
void handleBluetooth() {
  while (BT.available()) {
    char c = BT.read();
    switch (c) {
      case 'w': case 'W': case 's': case 'S':
      case 'a': case 'A': case 'd': case 'D':
        btCmd = c; btActive = true; break;
      case 'l': case 'L': speedPWM = LOW_SPEED; break;
      case 'h': case 'H': speedPWM = HIGH_SPEED; break;
      case 'x': case 'X':
        btCmd = 'x'; btActive = false; stopAll(); break;
      default: break;  // ignore newlines etc.
    }
  }

  if (!btActive) return;
  switch (btCmd) {
    case 'w': case 'W': drive(speedPWM, speedPWM); break;
    case 's': case 'S': drive(-speedPWM, -speedPWM); break;
    case 'a': case 'A': drive(-speedPWM, speedPWM); break;
    case 'd': case 'D': drive(speedPWM, -speedPWM); break;
  }
}

// ---------- Serial commands (Pi UART + USB) ----------
void handleSerialLine(const String &line) {
  int comma = line.indexOf(',');
  if (comma < 0) return;
  int left = line.substring(0, comma).toInt();
  int right = line.substring(comma + 1).toInt();
  lastSerialCmd = millis();
  if (!btActive) drive(left, right);
}

// Collect characters from one port into lines
void readLines(Stream &port, String &buf) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n') {
      handleSerialLine(buf);
      buf = "";
    } else if (c != '\r' && buf.length() < 32) {
      buf += c;
    }
  }
}

void handleSerial() {
  readLines(Serial2, piLine);
  readLines(Serial, usbLine);

  // Watchdog: Pi stopped talking -> stop (unless driving by Bluetooth)
  if (!btActive && millis() - lastSerialCmd > SERIAL_TIMEOUT) {
    stopAll();
  }
}

// ===== Main Loop =====
void loop() {
  handleBluetooth();
  handleSerial();
  delay(2);
}
