/* car_bridge_route.ino - laptop BT -> Arduino -> motor board + ultrasonic safety.
   High-Speed 200 RPM Motor Edition with Detached Servo & Early 48cm Braking.

   Wiring:
     HC-05 TXD -> D10 (SoftwareSerial RX)
     HC-05 RXD <- D11 (SoftwareSerial TX, via 1k resistor divider)
     Motor Board IN <- D3 (SoftwareSerial TX)
     HC-SR04 Trig -> D8
     HC-SR04 Echo -> D7
     Servo Signal -> D9 (detaches immediately after move to protect pulseIn)
     HC-SR04 VCC -> 5V
     HC-SR04 GND -> GND

   Modes:
     'M' -> Manual mode + Stop. Drive digits (8/2/4/6/5) pass through.
     'A' -> Freeplay autonomous obstacle avoidance around classroom.
     'R' -> Route / Maze Autopilot mode with forward ultrasonic safety guard.
*/

#include <SoftwareSerial.h>
#include <Servo.h>

#define BT_RX     10
#define BT_TX     11
#define MOTOR_RX   4
#define MOTOR_TX   3
#define TRIG_PIN   8
#define ECHO_PIN   7
#define SERVO_PIN  9

#define BT_BAUD     9600
#define MOTOR_BAUD  9600
#define DEBUG_BAUD  115200

// Calibrated for measured speed: ~30 cm/s
#define OBSTACLE_CM     36    // Confirmed obstacle braking distance (~34cm final stop after 70ms active brake)
#define SIDE_CLEAR_CM   28    // Minimum side clearance to steer
#define MAX_DIST_CM     400

#define SERVO_CENTRE    90
#define SERVO_LEFT      35
#define SERVO_RIGHT     145

// Calibrated: 1-motor pivot 960 ms per 90 deg
#define PING_MS         60
#define REVERSE_MS      600   // ~18 cm reverse at 30 cm/s
#define TURN_MS         960   // Exactly 90 deg pivot (1-motor drive)
#define TURN_AROUND_MS  1920  // 180 deg pivot (2 x 960 ms)

SoftwareSerial bt(BT_RX, BT_TX);
SoftwareSerial motor(MOTOR_RX, MOTOR_TX);
Servo steer;

bool autoMode = false;
bool routeMode = false;
bool routeMovingFwd = false;
bool routeBlocked = false;
unsigned long lastRoute = 0;
unsigned long lastAutoPing = 0;

// Watchdog for loose ultrasonic sensor wire
int faultZeroCount = 0;
int faultTimeoutCount = 0;
unsigned long lastFaultAlert = 0;

// Move servo and detach to release Timer1 so pulseIn() is never corrupted!
void moveServo(int angle, int waitMs = 300) {
  steer.attach(SERVO_PIN);
  steer.write(angle);
  delay(waitMs);
  steer.detach(); // Frees Timer1 interrupts for accurate pulseIn!
}

long measureCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // 22000 us = ~3.8 meters max distance. Avoids long 35ms stall when road is clear
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 22000UL);
  if (dur == 0) return MAX_DIST_CM;  // no echo or timeout = clear
  long cm = (long)(dur / 58);
  return cm;
}

int readScan() {
  long d1 = measureCm();
  if (d1 < OBSTACLE_CM) {
    // If d1 detects an obstacle, take a fast 2nd reading 15ms later to confirm.
    // This rejects single-sample motor noise, floor glitch, or sensor spikes ("too much caution").
    delay(15);
    long d2 = measureCm();
    if (d2 < OBSTACLE_CM) {
      return (int)min(d1, d2);  // Confirmed real obstacle
    } else {
      return (int)d2;          // Transient glitch discarded, road is clear
    }
  }
  return (int)d1;              // Road is clear, returns immediately
}

void checkUltrasonicFault(long cm, unsigned long now) {
  if (cm == 0) {
    faultZeroCount++;
  } else {
    faultZeroCount = 0;
  }

  if (cm >= MAX_DIST_CM) {
    faultTimeoutCount++;
  } else {
    faultTimeoutCount = 0;
  }

  // 8 consecutive readings stuck at 0
  if (faultZeroCount >= 8) {
    if (now - lastFaultAlert > 2000) {
      lastFaultAlert = now;
      Serial.println("FAULT:ULTRASONIC_LOOSE:STUCK_0");
      bt.println("FAULT:ULTRASONIC_LOOSE:STUCK_0");
    }
  }
}

void motorSend(char c) {
  motor.write(c);
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(DEBUG_BAUD);
  bt.begin(BT_BAUD);
  motor.begin(MOTOR_BAUD);
  bt.listen();

  motorSend('5');
  moveServo(SERVO_CENTRE, 300);
  Serial.println("RC Car Bridge + ROUTE Ready. 200 RPM Mode.");
}

void enterRoute() {
  autoMode = false;
  routeMode = true;
  routeMovingFwd = false;
  routeBlocked = false;
  motorSend('5');
  moveServo(SERVO_CENTRE, 250);
  Serial.println("MODE:ROUTE");
  bt.println("MODE:ROUTE");
}

void setModeAuto(bool toAuto) {
  routeMode = false;
  routeBlocked = false;
  routeMovingFwd = false;
  autoMode = toAuto;
  motorSend('5');

  if (toAuto) {
    moveServo(SERVO_CENTRE, 300);
  } else {
    steer.detach();
  }

  lastAutoPing = millis();
  Serial.println(toAuto ? "MODE:AUTO" : "MODE:MANUAL");
  bt.println(toAuto ? "MODE:AUTO" : "MODE:MANUAL");
}

void avoidObstacle() {
  motorSend('5'); // Immediate brake
  delay(100);

  // 1. Look Left
  moveServo(SERVO_LEFT, 350);
  int leftCm = readScan();

  // 2. Look Right
  moveServo(SERVO_RIGHT, 450);
  int rightCm = readScan();

  // 3. Return to Center
  moveServo(SERVO_CENTRE, 300);

  Serial.print("Left: "); Serial.print(leftCm);
  Serial.print(" cm | Right: "); Serial.print(rightCm); Serial.println(" cm");

  // Steer around obstacle with corner trap escape
  if (leftCm < SIDE_CLEAR_CM && rightCm < SIDE_CLEAR_CM) {
    // Corner trap: reverse firmly & 180 spin
    motorSend('2');
    delay(REVERSE_MS * 2);
    motorSend(leftCm > rightCm ? '6' : '4');
    delay(TURN_AROUND_MS);
  } else {
    // Normal obstacle: reverse slightly, turn to clearer side
    motorSend('2');
    delay(REVERSE_MS);
    motorSend(leftCm > rightCm ? '6' : '4'); // 6=Left, 4=Right
    delay(TURN_MS);
  }

  motorSend('5');
  delay(100);
}

void loop() {
  while (bt.available()) {
    char c = (char)bt.read();
    if (c == 'A' || c == 'a') {
      setModeAuto(true);
      continue;
    }
    if (c == 'M' || c == 'm') {
      setModeAuto(false);
      continue;
    }
    if (c == 'R' || c == 'r') {
      enterRoute();
      continue;
    }
    if (c == '8' || c == '2' || c == '4' || c == '6' || c == '5') {
      if (routeMode) {
        if (c == '8' && routeBlocked) {
          // Blocked by forward obstacle
          continue;
        }
        if (c == '5' || c == '2' || c == '4' || c == '6') {
          routeBlocked = false;  // Turn/Stop/Reverse clears route latch
        }
        routeMovingFwd = (c == '8');
        motorSend(c);
        continue;
      }
      if (autoMode) {
        setModeAuto(false);
      }
      routeMovingFwd = false;
      motorSend(c);
      continue;
    }
  }

  unsigned long now = millis();

  // ROUTE MODE: Fast obstacle guard while moving forward
  if (routeMode) {
    if (routeMovingFwd && !routeBlocked && (now - lastRoute >= PING_MS)) {
      lastRoute = now;
      long d = readScan();
      checkUltrasonicFault(d, now);

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
        motorSend('2');
        delay(70);
        motorSend('5');
        Serial.print("BLOCKED:");
        Serial.println(d);
        bt.print("BLOCKED:");
        bt.println(d);
      }
    }
    return;
  }

  // AUTO FREEPLAY MODE: Roam classroom and avoid obstacles
  if (autoMode && (now - lastAutoPing >= PING_MS)) {
    lastAutoPing = now;
    int d = readScan();
    checkUltrasonicFault(d, now);

    bt.print("cm=");
    bt.println(d);

    if (d < OBSTACLE_CM) {
      avoidObstacle();
      lastAutoPing = millis();
      return;
    }

    // Path is clear -> drive forward
    motorSend('8');
  }
}
