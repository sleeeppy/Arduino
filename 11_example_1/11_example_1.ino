#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm) = 18cm
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm) = 36cm

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

// 0.3: a single noisy sample moves EMA by 30% (narrow-range jitter stays small),
// while a real step reaches ~90% in about 7 samples (~175ms).
#define _EMA_ALPHA 0.3

// A real target moves well under this in 25ms. A false short echo is ~100mm.
#define MAX_STEP 40.0

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
float dist_pending;
int pending_ready = 0;
unsigned long last_sampling_time;       // unit: ms
int ema_ready = 0;

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH); // LED OFF

  myservo.attach(PIN_SERVO);
  myservo.write(90);

  dist_prev = _DIST_MAX;

  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  float dist_raw, dist_filtered;

  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // range filter: 18~36cm. timeout/out-of-range keeps the previous value
  // so a failed read (0) does not snap the servo to 0 deg.
  // A single in-range jump larger than MAX_STEP is also held. Two samples
  // that agree are accepted, so a real move still gets through in 50ms.
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
    dist_filtered = dist_prev;
    pending_ready = 0;
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else if (ema_ready && fabs(dist_raw - dist_prev) > MAX_STEP) {
    if (pending_ready && fabs(dist_raw - dist_pending) <= MAX_STEP) {
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
      pending_ready = 0;
      digitalWrite(PIN_LED, LOW);
    } else {
      dist_pending = dist_raw;
      pending_ready = 1;
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, HIGH);
    }
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    pending_ready = 0;
    digitalWrite(PIN_LED, LOW);  // LED ON
  }

  if (ema_ready == 0) {
    dist_ema = dist_filtered;
    ema_ready = 1;
  } else {
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;
  }

  // 18cm -> 0 deg, 36cm -> 180 deg, linear in between (1mm == 1 deg)
  // 19cm -> 10, 27cm -> 90, 35cm -> 170
  int angle;
  if (dist_ema <= _DIST_MIN) {
    angle = 0;
  } else if (dist_ema >= _DIST_MAX) {
    angle = 180;
  } else {
    angle = (int)(dist_ema - _DIST_MIN + 0.5);
  }
  myservo.write(angle);

  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",dist:");   Serial.print(dist_raw > _DIST_MAX + 80 ? _DIST_MAX + 80 : dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",Servo:");  Serial.print(myservo.read());
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
