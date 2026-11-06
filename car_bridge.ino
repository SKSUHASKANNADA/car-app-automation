/*
  ==============================================================
  Smart RC Car Bridge with Servo and Ultrasonic Obstacle Avoidance
  High-Speed 200 RPM Edition: Two-stage early braking & corner escape
  ==============================================================

  Motor Board Commands:
    '8' = Forward
    '2' = Backward
    '4' = Right
    '6' = Left
    '5' = Stop

  Bluetooth Modes:
    'M' = Manual Mode
    'A' = Automatic Obstacle Avoidance Mode (Freeplay)

  Wiring:
    HC-05 TXD        -> Arduino D10 (SoftwareSerial RX)
    HC-05 RXD        <- Arduino D11 (SoftwareSerial TX)
    Motor Board IN   <- Arduino D3  (SoftwareSerial TX to motor controller)
    HC-SR04 Trig     -> Arduino D8
    HC-SR04 Echo     -> Arduino D7
    Servo Signal     -> Arduino D9
    HC-SR04 VCC      -> 5V
    HC-SR04 GND      -> GND
*/

#include <SoftwareSerial.h>
#include <Servo.h>

// ================= PIN CONFIGURATION =================
const uint8_t PIN_TRIG = 8;
const uint8_t PIN_ECHO = 7;

const uint8_t PIN_BT_RX = 10;
const uint8_t PIN_BT_TX = 11;

const uint8_t PIN_BOARD_TX = 3;
const uint8_t PIN_BOARD_RX_UNUSED = 4;

const uint8_t PIN_SERVO = 9;

// ================= BAUD RATES =================
const long BT_BAUD = 9600;
const long BOARD_BAUD = 9600;

// ================= DISTANCE SETTINGS (Calibrated for measured speed: ~30 cm/s) =================
const int OBSTACLE_CM = 36;    // Confirmed obstacle braking distance (~34cm final stop after 70ms active brake)
const int SIDE_CLEAR_CM = 28;
const int MAX_DISTANCE_CM = 400;

// ================= SERVO SETTINGS =================
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 35;
const int SERVO_RIGHT  = 145;

// ================= TIMING SETTINGS (Calibrated: 1-motor pivot 960 ms per 90 deg) =================
const unsigned long PING_MS = 60;
const unsigned long REVERSE_MS = 600;       // ~18 cm reverse at 30 cm/s
const unsigned long TURN_MS = 960;          // Exactly 90 deg turn (single motor pivot)
const unsigned long TURN_AROUND_MS = 1920;  // 180 deg turn (2 x 960 ms)

const bool DEBUG = true;

// ================= OBJECTS =================
SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);
SoftwareSerial board(PIN_BOARD_RX_UNUSED, PIN_BOARD_TX);
Servo scanServo;

// ================= VARIABLES =================
bool autoMode = false;
bool routeMode = false;
bool routeMovingFwd = false;
bool routeBlocked = false;
char lastSent = 0;
unsigned long lastSendMs = 0;
unsigned long lastPingMs = 0;
unsigned long lastRouteMs = 0;

void sendToBoard(char c, bool force = false) {
  unsigned long now = millis();
  if (force || c != lastSent || now - lastSendMs >= 250) {
    board.write(c);
    lastSent = c;
    lastSendMs = now;
  }
}

// Servo helper: move and detach so Timer1 interrupts never corrupt pulseIn
void moveServo(int angle, int waitTimeMs = 300) {
  scanServo.attach(PIN_SERVO);
  scanServo.write(angle);
  delay(waitTimeMs);
  scanServo.detach(); // Free Timer1 for accurate pulseIn reading!
}

// ================= READ ULTRASONIC SENSOR =================
int readCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, 35000UL);
  if (us == 0) {
    return MAX_DISTANCE_CM; // No echo = clear
  }
  return (int)(us / 58);
}

int readScan() {
  int d1 = readCm();
  if (d1 < OBSTACLE_CM) {
    delay(15);
    int d2 = readCm();
    if (d2 < OBSTACLE_CM) {
      return min(d1, d2);
    } else {
      return d2;
    }
  }
  return d1;
}

// ================= MODE SWITCHING =================
void setMode(bool toAuto) {
  autoMode = toAuto;
  sendToBoard('5', true); // Stop immediately

  if (toAuto) {
    moveServo(SERVO_CENTER, 300);
  } else {
    scanServo.detach();
  }

  lastPingMs = millis();
  bt.println(toAuto ? "MODE:AUTO" : "MODE:MANUAL");
  if (DEBUG) {
    Serial.println(toAuto ? "MODE:AUTO" : "MODE:MANUAL");
  }
}

// ================= HANDLE BLUETOOTH INPUT =================
void handleBluetooth() {
  while (bt.available()) {
    char c = (char)bt.read();

    if (c == 'R' || c == 'r') {
      autoMode = false;
      routeMode = true;
      routeMovingFwd = false;
      routeBlocked = false;
      sendToBoard('5', true);
      moveServo(SERVO_CENTER, 250);
      Serial.println("MODE:ROUTE");
      bt.println("MODE:ROUTE");
      continue;
    }
    else if (c == 'A' || c == 'a' || c == 'O' || c == 'o') {
      routeMode = false;
      if (!autoMode) setMode(true);
    } 
    else if (c == 'M' || c == 'm' || c == 'B' || c == 'b') {
      routeMode = false;
      setMode(false);
      sendToBoard('5', true);
    } 
    else if (c == '8' || c == '2' || c == '4' || c == '6' || c == '5') {
      if (routeMode) {
        if (c == '8' && routeBlocked) {
          continue;
        }
        if (c == '5' || c == '2' || c == '4' || c == '6') {
          routeBlocked = false;
        }
        routeMovingFwd = (c == '8');
        sendToBoard(c, true);
        continue;
      }
      if (autoMode) {
        setMode(false);
      }
      routeMovingFwd = false;
      sendToBoard(c, true);
    }
  }
}

bool waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    handleBluetooth();
    if (!autoMode) return false;
    delay(5);
  }
  return true;
}

// ================= OBSTACLE AVOIDANCE =================
void avoid() {
  sendToBoard('5', true); // Immediate brake
  if (!waitMs(120)) return;

  // 1. Look Left
  moveServo(SERVO_LEFT, 350);
  int leftCm = readScan();

  // 2. Look Right
  moveServo(SERVO_RIGHT, 450);
  int rightCm = readScan();

  // 3. Return sensor to Center
  moveServo(SERVO_CENTER, 300);

  if (DEBUG) {
    Serial.print("Left: "); Serial.print(leftCm);
    Serial.print(" cm | Right: "); Serial.print(rightCm);
    Serial.println(" cm");
  }

  // 4. Steer around obstacle with corner-trap detection
  if (leftCm < SIDE_CLEAR_CM && rightCm < SIDE_CLEAR_CM) {
    // Dead end / corner trap: back up firmly & 180 turn
    sendToBoard('2', true);
    if (!waitMs(REVERSE_MS * 2)) return;

    // Turn toward whichever side has slightly more room
    sendToBoard(leftCm > rightCm ? '6' : '4', true);
    if (!waitMs(TURN_AROUND_MS)) return;
  } else {
    // Normal obstacle: reverse slightly, turn to open side
    sendToBoard('2', true);
    if (!waitMs(REVERSE_MS)) return;

    // '6' = Left, '4' = Right
    sendToBoard(leftCm > rightCm ? '6' : '4', true);
    if (!waitMs(TURN_MS)) return;
  }

  sendToBoard('5', true); // Settle
  waitMs(120);
}

// ================= AUTO NAVIGATION LOOP =================
void autoStep() {
  int d = readScan();

  bt.print("cm=");
  bt.println(d);
  if (DEBUG) {
    Serial.print("cm=");
    Serial.println(d);
  }

  // Obstacle detected early!
  if (d < OBSTACLE_CM) {
    avoid();
    lastPingMs = millis();
    return;
  }

  // Road clear -> Drive forward
  sendToBoard('8', false);
}

// ================= ARDUINO SETUP =================
void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  if (DEBUG) Serial.begin(115200);

  board.begin(BOARD_BAUD);
  bt.begin(BT_BAUD);
  bt.listen();

  // Initialize servo to center and detach
  moveServo(SERVO_CENTER, 300);

  sendToBoard('5', true); // Ensure car starts stationary
  Serial.println("RC Car Freeplay Ready. 200 RPM Mode.");
}

// ================= MAIN LOOP =================
void loop() {
  handleBluetooth();

  unsigned long now = millis();

  // ROUTE MODE: Fast ultrasonic obstacle guard during map navigation
  if (routeMode) {
    if (routeMovingFwd && !routeBlocked && (now - lastRouteMs >= PING_MS)) {
      lastRouteMs = now;
      int d = readScan();

      // Periodically stream cm reading so Python HUD stays live
      static unsigned long lastRouteTelemetry = 0;
      if (now - lastRouteTelemetry >= 180) {
        lastRouteTelemetry = now;
        bt.print("cm=");
        bt.println(d);
      }

      if (d < OBSTACLE_CM) {
        routeMovingFwd = false;
        routeBlocked = true;
        // Active reverse pulse stops forward momentum immediately (eliminates coasting)
        sendToBoard('2', true);
        delay(70);
        sendToBoard('5', true);
        if (DEBUG) {
          Serial.print("BLOCKED:");
          Serial.println(d);
        }
        bt.print("BLOCKED:");
        bt.println(d);
      }
    }
    return;
  }

  if (autoMode && (now - lastPingMs >= PING_MS)) {
    lastPingMs = now;
    autoStep();
  }
}
