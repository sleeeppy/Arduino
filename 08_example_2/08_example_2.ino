// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)
#define _DIST_PEAK 200.0  // max LED brightness (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar
  analogWrite(PIN_LED, 255);    // LED OFF (active low)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance;
  float brightness;  // 0.0 = 꺼짐, 1.0 = 최대밝기
  int pwm;           // analogWrite 값 (0=가장 밝음, 255=꺼짐)

  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((distance == 0.0) || (distance > _DIST_MAX)) {
    distance = _DIST_MAX + 10.0;
    brightness = 0.0;
  } else if (distance < _DIST_MIN) {
    distance = _DIST_MIN - 10.0;
    brightness = 0.0;
  } else if (distance <= _DIST_PEAK) {
    // 100mm -> 0, 150mm -> 0.5, 200mm -> 1.0
    brightness = (distance - _DIST_MIN) / (_DIST_PEAK - _DIST_MIN);
  } else {
    // 200mm -> 1.0, 250mm -> 0.5, 300mm -> 0
    brightness = (_DIST_MAX - distance) / (_DIST_MAX - _DIST_PEAK);
  }

  // active low: brightness 1.0 -> 0, brightness 0.0 -> 255
  pwm = (int)((1.0 - brightness) * 255.0 + 0.5);
  analogWrite(PIN_LED, pwm);

  // output the distance / brightness to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
//  Serial.print(",brightness:"); Serial.print(brightness);
//  Serial.print(",pwm:");       Serial.print(pwm);
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
