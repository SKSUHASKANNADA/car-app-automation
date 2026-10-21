/*
  car_bridge.ino - laptop BT -> Arduino -> motor board + ultrasonic auto-stop

  Laptop app (COM3 @9600) --> HC-05 --TXD--> D10 [ARDUINO] D3 --> motor board input
                                                     D8,D7 --> ultrasonic Trig,Echo

  Upload: Arduino IDE -> New Sketch -> paste all -> Upload
  Test: keep USB plugged in, Serial Monitor @115200 (NOT 9600)

  Manual: F/B/L/R/S in app send 8/2/4/6/5 -> forwarded silently to motor board
  Auto:   Mode button sends 'A' -> car drives forward alone, stops <25cm,
          restarts when clear >35cm. Any drive key sends digit -> back to manual.
          'M' -> manual + stop.
*/

#include <SoftwareSerial.h>

// ---- pins (same as your working wiring) ----
#define BT_RX     10   // HC-05 TXD -> D10 (Arduino receives laptop commands)
#define BT_TX     11   // D11 -> 1k -> HC-05 RXD (+2k RXD->GND). Optional, for msgs back to app
#define MOTOR_RX   4   // dummy RX, leave D4 empty (needed by SoftwareSerial)
#define MOTOR_TX   3   // D3 -> motor board input (where HC-05 TXD used to go)
#define TRIG_PIN   8   // ultrasonic Trig
#define ECHO_PIN   7   // ultrasonic Echo

#define BT_BAUD     9600
#define MOTOR_BAUD  9600
#define DEBUG_BAUD  115200

#define STOP_DIST   25   // cm: closer than this -> STOP
#define GO_DIST     35   // cm: clearer than this -> GO (hysteresis band 25-35 keeps last state)
#define AUTO_MS     150  // sensor check interval in AUTO

SoftwareSerial bt(BT_RX, BT_TX);       // laptop link
SoftwareSerial motor(MOTOR_RX, MOTOR_TX); // motor board link (TX only used)

bool autoMode = false;
bool autoGo = true;
unsigned long lastAuto = 0;

long measureCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // 30ms timeout ~ 500cm max. 0 = no echo = treat as clear.
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (dur == 0) return 999;
  return (long)(dur / 58);
}

void motorSend(char c) {
  motor.write(c);
}

void setManual(const char* why) {
  if (autoMode) {
    autoMode = false;
    Serial.println("MODE:MANUAL");
    bt.println("MODE:MANUAL");
  }
  // caller forwards the digit itself
  (void)why;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(DEBUG_BAUD);
  bt.begin(BT_BAUD);
  motor.begin(MOTOR_BAUD);
  bt.listen();  // keep BT as the RX listener (motor is TX-only so no conflict)

  motorSend('5'); // safety stop
  Serial.println("Bridge ready. MANUAL. BT@9600 USB@115200");
  Serial.println("Hold F in app = number should NOT print here in manual (silent forward).");
  Serial.println("Press Mode in app = expect MODE:AUTO then cm=.. GO/STOP lines.");
}

void loop() {
  // ---- 1) commands from laptop via HC-05 ----
  while (bt.available()) {
    char c = (char)bt.read();

    if (c == 'A') {
      autoMode = true;
      autoGo = true;
      lastAuto = 0; // force immediate reading
      Serial.println("MODE:AUTO");
      bt.println("MODE:AUTO");
      continue;
    }
    if (c == 'M') {
      autoMode = false;
      motorSend('5');
      Serial.println("MODE:MANUAL");
      bt.println("MODE:MANUAL");
      continue;
    }
    if (c == '8' || c == '2' || c == '4' || c == '6' || c == '5') {
      if (autoMode) {
        // any drive press (incl. S) drops back to manual = emergency stop path
        autoMode = false;
        Serial.println("MODE:MANUAL");
        bt.println("MODE:MANUAL");
      }
      motorSend(c); // silent in manual (no Serial spam)
      continue;
    }
    // ignore \r \n and anything else
  }

  // ---- 2) autonomous forward/stop ----
  if (!autoMode) return;

  unsigned long now = millis();
  if (now - lastAuto < AUTO_MS) return;
  lastAuto = now;

  long cm = measureCm();

  bool wantGo = autoGo;
  if (cm < STOP_DIST) wantGo = false;        // too close (note: 999 never < 25)
  else if (cm > GO_DIST) wantGo = true;      // clear (999 > 35 = go)
  // else 25..35: keep previous state (hysteresis, no chatter)

  if (wantGo != autoGo) {
    autoGo = wantGo;
    motorSend(autoGo ? '8' : '5');
    Serial.print("cm=");
    Serial.print(cm);
    Serial.println(autoGo ? "  GO" : "  STOP");
  } else {
    // refresh motor + heartbeat log every ~1s so you know it is alive
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
