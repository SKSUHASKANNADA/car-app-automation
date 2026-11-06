/*
  Ultrasonic Sensor (HC-SR04) Test Sketch
  
  Wiring:
    VCC  -> 5V (Not 3.3V!)
    GND  -> GND
    TRIG -> D8
    ECHO -> D7

  Open Serial Monitor at 115200 baud.
*/

const uint8_t PIN_TRIG = 8;
const uint8_t PIN_ECHO = 7;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.println("=====================================");
  Serial.println("HC-SR04 Ultrasonic Sensor Test");
  Serial.println("Pins: Trig=D8, Echo=D7, VCC=5V, GND=GND");
  Serial.println("Move your hand in front of the sensor");
  Serial.println("=====================================");
  delay(1000);
}

void loop() {
  // 1. Clear trig line
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);

  // 2. Send 10 microsecond pulse
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // 3. Read the duration of the echo pulse (timeout 35ms = ~600cm)
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 35000UL);

  if (duration == 0) {
    Serial.println("NO ECHO received! (Check: VCC to 5V, GND, Trig=D8, Echo=D7)");
  } else {
    long cm = duration / 58;
    long inches = duration / 148;

    Serial.print("Distance: ");
    Serial.print(cm);
    Serial.print(" cm  |  ");
    Serial.print(inches);
    Serial.print(" inches  (pulse: ");
    Serial.print(duration);
    Serial.println(" us)");
  }

  delay(250); // Read 4 times per second
}
