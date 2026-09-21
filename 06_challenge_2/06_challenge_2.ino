#define PIN_LED 7

// 1 = 10ms,  2 = 1ms,  3 = 0.1ms
#define PERIOD_SELECT 3

const int PERIODS_US[] = {10000, 1000, 100};

int period;
int duty;

void set_period(int p)
{
  period = p;
}

void set_duty(int d)
{
  duty = d;
}

// 5V - 220ohm - LED - GPIO7
// 핀이 LOW일 때 LED ON (sink)
void pwm_pulse()
{
  unsigned long t_on = (unsigned long)period * duty / 100;
  unsigned long t_off = (unsigned long)period - t_on;

  if (t_on > 0) {
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds(t_on);
  }
  if (t_off > 0) {
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(t_off);
  }
}

// 한 duty를 약 10ms 유지 → 0~100 101단계가 약 1초
void hold_10ms()
{
  int n = 10000 / period;
  for (int i = 0; i < n; i++) {
    pwm_pulse();
  }
}

// 그래프 0~1~2초: 최소 → 최대(1초) → 최소(1초)
void triangle()
{
  for (duty = 0; duty <= 100; duty++) {
    set_duty(duty);
    hold_10ms();
  }
  for (duty = 99; duty >= 0; duty--) {
    set_duty(duty);
    hold_10ms();
  }
}

void blink_n(int n)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(200);
    digitalWrite(PIN_LED, HIGH);
    delay(200);
  }
  delay(800);
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  set_period(PERIODS_US[PERIOD_SELECT - 1]);
  blink_n(PERIOD_SELECT);
}

void loop()
{
  triangle();
}
