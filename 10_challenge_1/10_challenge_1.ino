#include <Servo.h>
#include <math.h>

#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_TRIG  12
#define PIN_ECHO  13

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _DIST_MIN 50.0
#define _DIST_MAX 400.0
#define _EMA_ALPHA 0.5
#define DIST_CAR   200.0   // 이하면 차량 감지
#define DIST_CLEAR 280.0   // 이보다 멀면 지나감

#define BAR_DOWN 0         // 차단기 내림
#define BAR_UP   90        // 차단기 올림
#define MOVING_TIME 1500   // 한 번 올리고 내리는 시간 (ms)

// 1 = sigmoid,  2 = cosine ease-in-out
#define FUNC_SELECT 2
#define SIGMOID_K 8.0

Servo myServo;

unsigned long last_sampling_time;
float dist_prev = _DIST_MAX;
float dist_ema;
int ema_ready = 0;

int car_present = 0;
int bar_up = 0;
int moving = 0;
int startAngle;
int stopAngle;
unsigned long moveStartTime;

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}

float sigmoid_raw(float x)
{
  return 1.0 / (1.0 + exp(-x));
}

// t: 0~1 → 0~1, 양 끝이 더 완만
float ease_sigmoid(float t)
{
  float y0 = sigmoid_raw(-SIGMOID_K);
  float y1 = sigmoid_raw(SIGMOID_K);
  float y = sigmoid_raw(SIGMOID_K * (2.0 * t - 1.0));
  return (y - y0) / (y1 - y0);
}

// t: 0~1 → 0~1, 가운데가 더 고르게 변함
float ease_cosine(float t)
{
  return 0.5 * (1.0 - cos(PI * t));
}

float ease(float t)
{
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  if (FUNC_SELECT == 2) {
    return ease_cosine(t);
  }
  return ease_sigmoid(t);
}

void start_move(int from_angle, int to_angle)
{
  startAngle = from_angle;
  stopAngle = to_angle;
  moveStartTime = millis();
  moving = 1;
}

void blink_n(int n)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(150);
    digitalWrite(PIN_LED, HIGH);
    delay(150);
  }
  delay(300);
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH);

  myServo.attach(PIN_SERVO);
  myServo.write(BAR_DOWN);
  delay(500);

  Serial.begin(57600);
  blink_n(FUNC_SELECT);
  Serial.print("FUNC_SELECT = ");
  Serial.println(FUNC_SELECT);

  last_sampling_time = millis();
}

void loop()
{
  if (moving) {
    unsigned long progress = millis() - moveStartTime;
    float t = (float)progress / (float)MOVING_TIME;
    if (t > 1.0) t = 1.0;

    float s = ease(t);
    int angle = (int)(startAngle + (stopAngle - startAngle) * s + 0.5);
    myServo.write(angle);

    if (progress >= MOVING_TIME) {
      myServo.write(stopAngle);
      bar_up = (stopAngle == BAR_UP);
      moving = 0;
    }
  }

  if (millis() < last_sampling_time + INTERVAL)
    return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  float dist_filtered;

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
    dist_filtered = dist_prev;
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  }

  if (ema_ready == 0) {
    dist_ema = dist_filtered;
    ema_ready = 1;
  } else {
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;
  }

  if (dist_ema <= DIST_CAR) {
    car_present = 1;
  } else if (dist_ema >= DIST_CLEAR) {
    car_present = 0;
  }

  if (!moving) {
    if (car_present && !bar_up) {
      start_move(BAR_DOWN, BAR_UP);
    } else if (!car_present && bar_up) {
      start_move(BAR_UP, BAR_DOWN);
    }
  }

  digitalWrite(PIN_LED, car_present ? LOW : HIGH);

  Serial.print("Min:"); Serial.print(_DIST_MIN);
  Serial.print(",ema:"); Serial.print(dist_ema);
  Serial.print(",car:"); Serial.print(car_present);
  Serial.print(",bar:"); Serial.print(bar_up);
  Serial.print(",Max:"); Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}
