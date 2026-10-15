#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

// measured on this servo (0 / 90 / 180 degree)
#define _DUTY_MIN 550
#define _DUTY_NEU 1475
#define _DUTY_MAX 2400

#define INTERVAL 20 // msec

// 13C02: 3 deg/s moves 180 deg in 60 s, 0.3 deg/s moves 90 deg in 300 s
#define _SERVO_SPEED_180 3.0f
#define _SERVO_SPEED_90  0.3f

unsigned long last_sampling_time;

Servo myservo;

float us_per_deg;
float duty_pos;  // fractional microseconds are kept
float duty_step; // change per interval, smaller than 1 us at these speeds
float duty_goal;
int phase;       // 1: 180 deg, 2: 90 deg, 3: done
int print_cnt;

void begin_move(float from_us, float to_us, float speed_dps) {
  float sign = (to_us >= from_us) ? 1.0f : -1.0f;

  duty_pos = from_us;
  duty_goal = to_us;
  duty_step = sign * us_per_deg * speed_dps * (INTERVAL / 1000.0f);
}

void setup() {
  us_per_deg = (float)(_DUTY_MAX - _DUTY_MIN) / 180.0f;

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);

  phase = 1;
  begin_move(_DUTY_MIN, _DUTY_MAX, _SERVO_SPEED_180);
  myservo.writeMicroseconds((int)(duty_pos + 0.5f));

  Serial.begin(57600);
  Serial.println("phase:1,speed:3,deg:0->180");

  last_sampling_time = millis();
  print_cnt = 0;
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL)
    return;
  last_sampling_time += INTERVAL;

  if (phase < 3) {
    duty_pos += duty_step;

    if ((duty_step > 0 && duty_pos >= duty_goal) ||
        (duty_step < 0 && duty_pos <= duty_goal)) {
      duty_pos = duty_goal;
      if (phase == 1) {
        phase = 2;
        begin_move(_DUTY_MAX, _DUTY_NEU, _SERVO_SPEED_90);
        Serial.println("phase:2,speed:0.3,deg:180->90");
      } else {
        phase = 3;
      }
    }

    myservo.writeMicroseconds((int)(duty_pos + 0.5f));
  }

  // One line per second while moving. A print every 20 ms would stretch the timing.
  if (phase < 3 && ++print_cnt >= 50) {
    print_cnt = 0;
    Serial.print("phase:");
    Serial.print(phase);
    Serial.print(",deg:");
    Serial.println((duty_pos - _DUTY_MIN) / us_per_deg);
  }
}
