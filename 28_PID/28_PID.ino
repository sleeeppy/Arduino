#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Debug mode
#define MODE   2  // 0: serial plotter, 1: serial monitor, 2: evaluation

// Event interval parameters
#define _INTERVAL_DIST    20     // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20     // servo interval (unit: ms)

//////////////// DO NOT modify below section!! //////////////////////////    
#define _INTERVAL_SERIAL  100    // serial interval (unit: ms)
#define _INTERVAL_MOVE    10000  // Target move interval (unit: ms)
/////////////////////////////////////////////////////////////////////////

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.4    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter

// Servo adjustment - Set _DUTY_MAX, _NEU, _MIN with your own numbers
#define _DUTY_MAX 2200 // rail end lowest (ball rolls away from the sensor)
#define _DUTY_NEU 1400 // roughly level
#define _DUTY_MIN 600  // rail end highest (ball rolls toward the sensor)

#define _SERVO_ANGLE_DIFF  144  // |D - E| degree: (2200 - 600) us * 0.09 deg/us
#define _SERVO_SPEED       450  // servo speed (deg/s) -> 100 us per 20 ms step

// PID parameters
#define _KP 15.0  // proportional gain (us per mm)
#define _KD 800.0 // derivative gain (us per mm/sample)
#define _KI 0.5   // integral gain (us per mm, per sample)

// Integrator: works only near the target, with a clamped error and a clamped sum (anti-windup)
#define _ITERM_ZONE   60    // integrate only while |error| < this (mm)
#define _ITERM_ERRMAX 10    // error fed to the integrator is clamped to +-this (mm)
#define _ITERM_MAX    700   // |iterm| limit (us)
#define _ITERM_DEAD   3.0   // do not integrate while |error| < this (mm): inside the allowed band, leave the ball alone
#define _ITERM_NEAR   0     // starting iterm for the 55 mm target (then remembered from the last visit)
#define _ITERM_FAR    0     // starting iterm for the 255 mm target (then remembered from the last visit)

// Derivative: low-pass filtered, full strength near the target and weaker far away (brake later)
#define _D_BETA     0.4     // EMA weight of the newest distance step
#define _D_NEAR     25      // full D gain while |error| <= this (mm)
#define _D_FAR      60      // scaled D gain while |error| >= this (mm)
#define _D_FAR_OUT  1.0     // D scale when the target is farther than the ball
#define _D_FAR_IN   0.45    // D scale when the target is nearer than the ball

// Sensor: a reading that jumps more than this from the estimate is ignored (servo noise spikes)
#define _GLITCH_MM    30
#define _GLITCH_HOLD  8     // ...for at most this many samples in a row
#define _IR_SAMPLES   100

// global variables

Servo myservo;      // Servo instance
float dist_ema;     // filtered distance

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial; // event triggered?

//////////////// DO NOT modify below section!! /////////////////////////        
unsigned long error_sum, error_cnt, toggle_cnt;
unsigned long last_sampling_time_move; // unit: msec
float dist_target = 55;

#define _TIME_TO_STABILIZE  3000  // time to control 
#define _ERROR_ALLOWED      3     // allowable error (unit: mm)
////////////////////////////////////////////////////////////////////////

// Servo speed control
int duty_chg_per_interval;     // maximum duty difference per interval

// Servo position
int duty_target, duty_current;

// PID variables
int error_current, error_prev;
float pterm, dterm, iterm;
float d_filtered;                 // filtered distance step (mm per sample)
float target_prev = -1;           // to notice the target change
float iterm_mem[2] = {_ITERM_NEAR, _ITERM_FAR}; // iterm remembered per target
byte glitch_cnt;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  
  myservo.attach(PIN_SERVO);
  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  // convert angle speed into duty change per interval
  duty_chg_per_interval = 
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0); 

  // initialize serial port
  Serial.begin(1000000);
}

void loop() {
  unsigned long time_current = millis();
  
  // wait until next event time
  if (time_current >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_current >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_current >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

//////////////// DO NOT modify below section!! /////////////////////////        
  if (time_current >= (last_sampling_time_move + _INTERVAL_MOVE)) {
        last_sampling_time_move += _INTERVAL_MOVE;
        dist_target = 310 - dist_target;
        if (MODE == 2 && ++toggle_cnt == 5) {
          Serial.println("--------------------------------------------------");
          Serial.print("ERR_cnt:");
          Serial.print(error_cnt);
          Serial.print(",ERR_sum:");
          Serial.print(error_sum);
          Serial.print(",ERR_ave:");
          Serial.println(error_sum / (float) error_cnt);
          exit(0);
        }
  }
////////////////////////////////////////////////////////////////////////

  if (event_dist) {
    float dist_filtered; // unit: mm
    int control;
    
    event_dist = false;

    // get a distance reading from the distance sensor
    float dist_before = dist_ema;
    dist_filtered = volt_to_distance(ir_sensor_mean(_IR_SAMPLES));
    if (fabs(dist_filtered - dist_ema) > _GLITCH_MM && glitch_cnt < _GLITCH_HOLD && time_current > 500) {
      glitch_cnt++;  // implausible jump: keep the previous estimate
    } else {
      glitch_cnt = 0;
      dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;
    }

    // target changed: store the iterm of the old target, restore the one of the new target
    if (dist_target != target_prev) {
      if (target_prev >= 0)
        iterm_mem[target_prev < 155 ? 0 : 1] = iterm;
      iterm = iterm_mem[dist_target < 155 ? 0 : 1];
      target_prev = dist_target;
    }

    // PID control logic
    error_current = dist_target - dist_ema;

//////////////// DO NOT modify below section!! /////////////////////////
    if (time_current >= last_sampling_time_move + _TIME_TO_STABILIZE) {
      error_cnt++;
      if (abs(error_current) > _ERROR_ALLOWED) {
        error_sum += abs(error_current);
      }
    }
    if (abs(error_current) <= _ERROR_ALLOWED) {
      digitalWrite(PIN_LED, 0);      
    } else {
      digitalWrite(PIN_LED, 1);
    }
////////////////////////////////////////////////////////////////////////
    
    float error_f = dist_target - dist_ema;   // float error: no 1 mm steps in the P and I terms
    float error_abs = fabs(error_f);

    pterm = _KP * error_f;

    // D on the measured distance (no kick when the target jumps), filtered, gain scheduled by |error|
    d_filtered = _D_BETA * -(dist_ema - dist_before) + (1.0 - _D_BETA) * d_filtered;
    float d_scale = 1.0;
    if (error_abs > _D_NEAR) {
      float far_scale = (error_f > 0) ? _D_FAR_OUT : _D_FAR_IN;
      if (error_abs >= _D_FAR)
        d_scale = far_scale;
      else
        d_scale = 1.0 + (far_scale - 1.0) * (error_abs - _D_NEAR) / (_D_FAR - _D_NEAR);
    }
    dterm = _KD * d_scale * d_filtered;

    if (error_abs < _ITERM_ZONE && error_abs >= _ITERM_DEAD) {
      float error_i = error_f;
      if (error_i > _ITERM_ERRMAX)  error_i = _ITERM_ERRMAX;
      if (error_i < -_ITERM_ERRMAX) error_i = -_ITERM_ERRMAX;
      iterm += _KI * error_i;
    }
    if (iterm > _ITERM_MAX)  iterm = _ITERM_MAX;
    if (iterm < -_ITERM_MAX) iterm = -_ITERM_MAX;

    control = pterm + dterm + iterm;
    error_prev = error_current;

    duty_target = _DUTY_NEU + control;

    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN)
      duty_target = _DUTY_MIN; // lower limit
    if (duty_target > _DUTY_MAX)
      duty_target = _DUTY_MAX; // upper limit      
  }
  
  if (event_servo) {
    event_servo = false;

    // adjust duty_current toward duty_target by duty_chg_per_interval
    if (duty_target > duty_current) {
      duty_current += duty_chg_per_interval;
      if (duty_current > duty_target)
        duty_current = duty_target;
    } else {
      duty_current -= duty_chg_per_interval;
      if (duty_current < duty_target)
        duty_current = duty_target;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_current);    
  }

  //////////////// DO NOT modify below section!! /////////////////////////        
  if (event_serial) {
    event_serial = false;
    
    if (MODE == 0) { // use for debugging with serial plotter
      Serial.print("MIN:0,MAX:310,TARGET:"); Serial.print(dist_target);
      Serial.print(",DIST:");                Serial.print(dist_ema);
      Serial.print(",ERR_ave(10X):");        Serial.println(error_sum / (float) error_cnt * 10);
    }

    if (MODE == 1) { // use for debugging with serial monitor
      Serial.print("MIN:0,MAX:310,TARGET:"); Serial.print(dist_target);
      Serial.print(",DIST:");    Serial.print(dist_ema);
      Serial.print(",pterm:");   Serial.print(pterm);
      Serial.print(",dterm:");   Serial.print(dterm);
      Serial.print(",iterm:");   Serial.print(iterm);
      Serial.print(",ERR_cnt:"); Serial.print(error_cnt);
      Serial.print(",ERR_sum:"); Serial.print(error_sum);
 
      Serial.print(",ERR_current:"); Serial.print(abs(error_current));
      Serial.print(",ERR_ave:"); Serial.println(error_sum / (float) error_cnt);
    }

    if (MODE == 2) { // use for evaluation
      Serial.print("ERR_cnt:");
      Serial.print(error_cnt);
      Serial.print(",ERR_current:");
      Serial.print(abs(error_current));
      Serial.print(",ERR_sum:");
      Serial.print(error_sum);
      Serial.print(",ERR_ave:");
      Serial.println(error_sum / (float) error_cnt);
    }
  }
}
/////////////////////////////////////////////////////////////////////////

float volt_to_distance(float a_value)
{
  // Replace next line into your own equation
  // return (6762.0 / (a_value - 9) - 4.0) * 10.0; 
  return 992.5 - 5.459 * a_value + 1.061E-02 * a_value * a_value - 7.043E-06 * a_value * a_value * a_value; // 11-1 cubic fit
}

int compare(const void *a, const void *b);

float ir_sensor_mean(unsigned int n)
{
  // Mean of the middle half of n sorted samples: drops the spikes like a median does,
  // but keeps sub-count resolution (1 ADC count is about 2 mm at 255 mm).
  static unsigned int samples[100];
  unsigned long sum = 0;
  unsigned int lo = n / 4, hi = n - n / 4;

  if (n == 0 || n > 100)
    return 0;
  for (unsigned int i = 0; i < n; i++)
    samples[i] = analogRead(PIN_IR);
  qsort(samples, n, sizeof(unsigned int), compare);
  for (unsigned int i = lo; i < hi; i++)
    sum += samples[i];
  return (float)sum / (hi - lo);
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
  // verbose: 0 - normal operation, 1 - observing the internal procedures, and 2 - calculating elapsed time.
  // Example 1: ir_sensor_filtered(n, 0.5, 0) => return the median value among the n measured samples.
  // Example 2: ir_sensor_filtered(n, 0.0, 0) => return the smallest value among the n measured samples.
  // Example 3: ir_sensor_filtered(n, 1.0, 0) => return the largest value among the n measured samples.

  // The output of Sharp infrared sensor includes lots of spiky noise.
  // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
  // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
  // Step 2. Return the position'th sample after sorting the collected samples.

  // returns 0, if any error occurs

  unsigned int *ir_val, ret_val;
  unsigned int start_time;
 
  if (verbose >= 2)
    start_time = millis(); 

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;
    
  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", position: "); Serial.print(position); 
    Serial.print(", ret_idx: ");  Serial.println((unsigned int)(n * position)); 
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print("IR:");
  
  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print  ("  => ");

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
    Serial.print(" :: ");
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }
  
  return ret_val;
}
