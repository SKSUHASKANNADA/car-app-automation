/* car_bridge_route.ino - laptop BT -> Arduino -> motor board + ultrasonic safety.
   Copy of car_bridge.ino + new ROUTE mode 'R' for route_planner_app.py.
   Upload ONE sketch at a time: either car_bridge.ino (original) or this file.

   Wiring (same as car_bridge.ino):
     HC-05 TXD -> D10, D11 -> 1k -> HC-05 RXD (+2k RXD->GND) for replies back,
     D3 -> motor board input, D8/D7 ultrasonic Trig/Echo.
     Optional servo on D9 (route mode holds it centred at 90, no sweep).

   Modes:
     'M' -> manual + stop, replies MODE:MANUAL. Drive digits 8/2/4/6/5 pass through.
     'A' -> auto obstacle-avoidance (as before). Any drive digit drops to manual.
     'R' -> route mode, replies MODE:ROUTE. Servo centred, no sweep.
            Drive digits pass through as in manual, EXCEPT forward guard:
            ultrasonic read ~every 60 ms; while moving forward ('8'),
            if dist < ROUTE_STOP_CM (no echo = clear), send '5' to board,
            reply 'BLOCKED:<cm>' ONCE, ignore further '8' until next
            non-forward cmd (2/4/6/5) or a new 'R'.
*/
#include <SoftwareSerial.h>
#ifdef SERVO_PIN
#include <Servo.h>
#endif

#define BT_RX     10
#define BT_TX     11
#define MOTOR_RX   4
#define MOTOR_TX   3
#define TRIG_PIN   8
#define ECHO_PIN   7

#define BT_BAUD     9600
#define MOTOR_BAUD  9600
#define DEBUG_BAUD  115200

#define STOP_DIST   25
#define GO_DIST     35
#define AUTO_MS     150

#define ROUTE_STOP_CM 20
#define ROUTE_MS      60
#define SERVO_PIN     9
#define SERVO_CENTRE  90

SoftwareSerial bt(BT_RX, BT_TX);
SoftwareSerial motor(MOTOR_RX, MOTOR_TX);
#ifdef SERVO_PIN
Servo steer;
bool servoOn = false;
#endif

bool autoMode = false;
bool autoGo = true;
unsigned long lastAuto = 0;

bool routeMode = false;
bool routeMovingFwd = false;
bool routeBlocked = false;
unsigned long lastRoute = 0;

long measureCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (dur == 0) return 999;  // no echo = clear
  return (long)(dur / 58);
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
  Serial.println("Bridge+ROUTE ready. MANUAL. BT@9600 USB@115200");
}

void enterRoute() {
  autoMode = false;
  routeMode = true;
  routeMovingFwd = false;
  routeBlocked = false;
  lastRoute = 0;
  motorSend('5');
#ifdef SERVO_PIN
  if (!servoOn) { steer.attach(SERVO_PIN); servoOn = true; }
  steer.write(SERVO_CENTRE);  // centred, no sweep in route mode
#endif
  Serial.println("MODE:ROUTE");
  bt.println("MODE:ROUTE");
}

void loop() {
  while (bt.available()) {
    char c = (char)bt.read();
    if (c == 'A') {
      routeMode = false;
      routeBlocked = false;
      routeMovingFwd = false;
      autoMode = true;
      autoGo = true;
      lastAuto = 0;
      Serial.println("MODE:AUTO");
      bt.println("MODE:AUTO");
      continue;
    }
    if (c == 'M') {
      autoMode = false;
      routeMode = false;
      routeBlocked = false;
      routeMovingFwd = false;
      motorSend('5');
      Serial.println("MODE:MANUAL");
      bt.println("MODE:MANUAL");
      continue;
    }
    if (c == 'R') {
      enterRoute();
      continue;
    }
    if (c == '8' || c == '2' || c == '4' || c == '6' || c == '5') {
      if (routeMode) {
        // stay in route mode; forward guard with latch
        if (c == '8' && routeBlocked) {
          // ignore latched forward until turn/stop or new R
          continue;
        }
        if (c != '8') routeBlocked = false;  // non-forward clears latch
        routeMovingFwd = (c == '8');
        motorSend(c);
        continue;
      }
      if (autoMode) {
        autoMode = false;
        Serial.println("MODE:MANUAL");
        bt.println("MODE:MANUAL");
      }
      routeMovingFwd = false;
      motorSend(c);
      continue;
    }
  }

  unsigned long now = millis();
  if (routeMode) {
    if (now - lastRoute < ROUTE_MS) return;
    lastRoute = now;
    if (!routeMovingFwd || routeBlocked) return;
    long cm = measureCm();
    if (cm < ROUTE_STOP_CM) {  // 999 never < 20 -> clear
      routeMovingFwd = false;
      routeBlocked = true;
      motorSend('5');
      Serial.print("BLOCKED:");
      Serial.println(cm);
      bt.print("BLOCKED:");
      bt.println(cm);
    }
    return;
  }

  if (!autoMode) return;
  if (now - lastAuto < AUTO_MS) return;
  lastAuto = now;
  long cm = measureCm();
  bool wantGo = autoGo;
  if (cm < STOP_DIST) wantGo = false;
  else if (cm > GO_DIST) wantGo = true;
  if (wantGo != autoGo) {
    autoGo = wantGo;
    motorSend(autoGo ? '8' : '5');
    Serial.print("cm=");
    Serial.print(cm);
    Serial.println(autoGo ? "  GO" : "  STOP");
  } else {
    static unsigned long lastRefresh = 0;
    static unsigned long lastLog = 0;
    if (now - lastRefresh > 1000) {
      lastRefresh = now;
      motorSend(autoGo ? '8' : '5');
    }
    if (now - lastLog > 1000) {
      lastLog = now;
      Serial.print("cm=");
      Serial.print(cm);
      Serial.println(autoGo ? "  GO" : "  STOP");
    }
  }
}
