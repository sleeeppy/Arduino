// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// 1 = N 3,  2 = N 10,  3 = N 30
#define N_SELECT 3
const int MEDIAN_NS[] = {3, 10, 30};
#define N_MAX 30

// global variables
unsigned long last_sampling_time;
float dist_ema;
int n_samples;
int ema_ready = 0;

float samples[N_MAX];
int sample_idx = 0;
int sample_count = 0;

void add_sample(float value)
{
  samples[sample_idx] = value;
  sample_idx = (sample_idx + 1) % n_samples;
  if (sample_count < n_samples) {
    sample_count++;
  }
}

float median_filter()
{
  float buf[N_MAX];
  int n = sample_count;

  for (int i = 0; i < n; i++) {
    buf[i] = samples[i];
  }

  for (int i = 1; i < n; i++) {
    float key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }

  if (n % 2 == 1) {
    return buf[n / 2];
  }
  return (buf[n / 2 - 1] + buf[n / 2]) / 2.0;
}

void blink_n(int n)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(200);
    digitalWrite(PIN_LED, HIGH);
    delay(200);
  }
  delay(500);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH);

  n_samples = MEDIAN_NS[N_SELECT - 1];
  Serial.begin(57600);
  blink_n(N_SELECT);
  last_sampling_time = millis();
}

void loop() {
  float dist_raw, dist_median;

  if (millis() < last_sampling_time + INTERVAL)
    return;

  // 직전 유효값 대체 없이 raw를 그대로 사용
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  add_sample(dist_raw);
  dist_median = median_filter();

  if (ema_ready == 0) {
    dist_ema = dist_raw;
    ema_ready = 1;
  } else {
    dist_ema = _EMA_ALPHA * dist_raw + (1.0 - _EMA_ALPHA) * dist_ema;
  }

  Serial.print("Min:"); Serial.print(_DIST_MIN);
  Serial.print(",raw:"); Serial.print(dist_raw);
  Serial.print(",ema:"); Serial.print(dist_ema);
  Serial.print(",median:"); Serial.print(dist_median);
  Serial.print(",Max:"); Serial.print(_DIST_MAX);
  Serial.println("");

  if ((dist_median < _DIST_MIN) || (dist_median > _DIST_MAX))
    digitalWrite(PIN_LED, 1);
  else
    digitalWrite(PIN_LED, 0);

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
