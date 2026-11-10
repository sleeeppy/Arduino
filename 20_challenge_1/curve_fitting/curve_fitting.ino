// 20C10 challenge: automatic curve fitting for the Sharp IR distance sensor.
//
// 1. Pick a fitting method and degree from the serial monitor.
// 2. Put the ping-pong ball at 0, 5, 10, ..., 30 cm and press Enter each time.
// 3. The sketch fits distance(mm) = f(ADC) by least squares and prints
//    a volt_to_distance() function that can be pasted into another sketch as-is.
//
// Serial monitor: 1000000 baud, line ending "Newline" (any setting except
// "No line ending" works, because an empty line must still send something).

#define PIN_IR A0

#define N_POINTS    7     // 0, 5, 10, 15, 20, 25, 30 cm
#define STEP_MM     50    // 5 cm between points
#define MAX_DEG     (N_POINTS - 1)
#define MEDIAN_N    31    // one median filter call reads 31 samples (~3.5ms)
#define AVG_ROUNDS  10    // average 10 medians: the ADC value gets a fraction part

#define METHOD_POLY 1     // dist = c0 + c1*x + c2*x^2 + ...          (x = ADC)
#define METHOD_INV  2     // dist = c0 + c1*u + c2*u^2 + ..., u = 1000/x
                          // Sharp sensors follow dist ~ k / ADC, so a low degree fits well.

float adc_val[N_POINTS];  // measured ADC value (average of medians)
float dist_mm[N_POINTS];  // real ball position

int method = METHOD_POLY;
int degree = 3;

// The fit is done on t = (feature(x) - f_mid) / f_half, which keeps t in [-1, 1].
// With raw ADC values x^6 reaches 1e18 and 32-bit float loses all digits.
float coef[MAX_DEG + 1];
float f_mid, f_half;
int fitted = 0;

char line[16];

void setup()
{
  Serial.begin(1000000);
  while (!Serial)
    ;
  Serial.println();
  Serial.println(F("==== IR sensor curve fitting (20C10) ===="));
  Serial.println(F("Serial monitor line ending: set to \"Newline\"."));
  ask_settings();
  measure_all();
  run_fit();
}

void loop()
{
  Serial.println();
  Serial.println(F("[f] refit with another method/degree  [m] measure again  [t] test distance"));
  read_line();

  if (line[0] == 'f') {
    ask_settings();
    run_fit();
  } else if (line[0] == 'm') {
    measure_all();
    run_fit();
  } else if (line[0] == 't') {
    test_mode();
  }
}

// ---------------------------------------------------------------- serial input

// Read one line into line[]. '\r' is skipped so "Both NL & CR" does not make a second
// empty line. With "Carriage return" there is no '\n', so 50ms of silence ends the line.
void read_line()
{
  int len = 0;
  unsigned long last;
  char c;

  while (Serial.available() == 0)
    ;
  last = millis();
  while (millis() - last < 50) {
    if (Serial.available() == 0)
      continue;
    c = Serial.read();
    last = millis();
    if (c == '\n')
      break;
    if (c == '\r' || c == ' ')
      continue;
    if (len < (int)sizeof(line) - 1)
      line[len++] = (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c;
  }
  line[len] = '\0';
}

int ask_int(const __FlashStringHelper *prompt, int lo, int hi, int def)
{
  int v;

  while (1) {
    Serial.print(prompt);
    Serial.print(F(" ["));
    Serial.print(lo); Serial.print('-'); Serial.print(hi);
    Serial.print(F(", Enter="));
    Serial.print(def);
    Serial.print(F("]: "));
    read_line();
    if (line[0] == '\0') {
      Serial.println(def);
      return def;
    }
    v = atoi(line);
    if (v >= lo && v <= hi) {
      Serial.println(v);
      return v;
    }
    Serial.println(F("out of range, try again"));
  }
}

void ask_settings()
{
  Serial.println();
  Serial.println(F("Fitting method"));
  Serial.println(F("  1: polynomial of ADC value    dist = c0 + c1*x + c2*x^2 + ..."));
  Serial.println(F("  2: polynomial of 1000/ADC     dist = c0 + c1*u + c2*u^2 + ..., u = 1000/x"));
  method = ask_int(F("method"), 1, 2, method);
  degree = ask_int(F("degree"), 1, MAX_DEG, degree);
  if (degree == N_POINTS - 1)
    Serial.println(F("note: 7 points and degree 6 -> the curve hits every point (R^2 = 1). Overfitting."));
}

// ---------------------------------------------------------------- measurement

// Average of AVG_ROUNDS medians. spread = largest median - smallest median.
float measure_adc(int rounds, unsigned int *spread)
{
  unsigned int v, lo = 1023, hi = 0;
  float sum = 0.0;

  for (int k = 0; k < rounds; k++) {
    v = ir_sensor_filtered(MEDIAN_N, 0.5, 0);
    sum += v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  *spread = hi - lo;
  return sum / rounds;
}

void measure_all()
{
  int i = 0;
  unsigned int spread;

  Serial.println();
  Serial.println(F("Measurement. Enter: measure,  r: redo the previous point"));

  // i == N_POINTS is the last chance to redo the 30 cm point before fitting.
  while (i <= N_POINTS) {
    if (i < N_POINTS) {
      Serial.print(F("[")); Serial.print(i + 1); Serial.print('/'); Serial.print(N_POINTS);
      Serial.print(F("] put the ball at "));
      Serial.print(i * STEP_MM / 10);
      Serial.print(F(" cm and press Enter"));
    } else {
      Serial.print(F("all points measured. Enter: run curve fitting"));
    }
    Serial.println();

    read_line();
    if (line[0] == 'r') {
      if (i > 0) {
        i--;
        Serial.println(F("  redo"));
      }
      continue;
    }
    if (i == N_POINTS)
      break;

    dist_mm[i] = i * STEP_MM;
    adc_val[i] = measure_adc(AVG_ROUNDS, &spread);
    Serial.print(F("  ADC = ")); Serial.print(adc_val[i], 1);
    Serial.print(F("  (median spread ")); Serial.print(spread); Serial.print(')');
    if (spread > 10)
      Serial.print(F("  <- unstable, check the ball and press r to redo"));
    Serial.println();
    i++;
  }

  // Tab separated: paste into the spreadsheet and the values land in columns A and B.
  Serial.println();
  Serial.println(F("Spreadsheet data (copy & paste into A1):"));
  Serial.println(F("ADC\tDistance(mm)"));
  for (i = 0; i < N_POINTS; i++) {
    Serial.print(adc_val[i], 1); Serial.print('\t'); Serial.println(dist_mm[i], 0);
  }
  fitted = 0;
}

// ---------------------------------------------------------------- fitting

float feature(float x)
{
  if (method == METHOD_INV) {
    if (x < 1.0)
      x = 1.0;
    return 1000.0 / x;
  }
  return x;
}

float eval_t(float t)
{
  float y = coef[degree];
  for (int k = degree - 1; k >= 0; k--)
    y = y * t + coef[k];
  return y;
}

float eval_fit(float x)
{
  return eval_t((feature(x) - f_mid) / f_half);
}

// Least squares with modified Gram-Schmidt QR on the N x (degree+1) matrix A[i][k] = t_i^k.
// QR does not square the condition number like the normal equations (A^T A) do,
// which matters with 32-bit float. Returns 0 if the columns are dependent.
int least_squares(const float *t, const float *y, int n, int m)
{
  float a[N_POINTS][MAX_DEG + 1];
  float r[MAX_DEG + 1][MAX_DEG + 1];
  float b[N_POINTS], z[MAX_DEG + 1];
  float s;
  int i, j, k;

  for (i = 0; i < n; i++) {
    a[i][0] = 1.0;
    for (k = 1; k < m; k++)
      a[i][k] = a[i][k - 1] * t[i];
    b[i] = y[i];
  }

  for (k = 0; k < m; k++) {
    s = 0.0;
    for (i = 0; i < n; i++)
      s += a[i][k] * a[i][k];
    r[k][k] = sqrt(s);
    if (r[k][k] < 1e-4)
      return 0;
    for (i = 0; i < n; i++)
      a[i][k] /= r[k][k];

    for (j = k + 1; j < m; j++) {
      s = 0.0;
      for (i = 0; i < n; i++)
        s += a[i][k] * a[i][j];
      r[k][j] = s;
      for (i = 0; i < n; i++)
        a[i][j] -= s * a[i][k];
    }

    s = 0.0;
    for (i = 0; i < n; i++)
      s += a[i][k] * b[i];
    z[k] = s;
    for (i = 0; i < n; i++)
      b[i] -= s * a[i][k];
  }

  // back substitution: R c = z
  for (k = m - 1; k >= 0; k--) {
    s = z[k];
    for (j = k + 1; j < m; j++)
      s -= r[k][j] * coef[j];
    coef[k] = s / r[k][k];
  }
  return 1;
}

void run_fit()
{
  float t[N_POINTS], f, f_lo, f_hi, mean, ss_res, ss_tot, err, r2, rmse;
  int i;

  f_lo = f_hi = feature(adc_val[0]);
  for (i = 1; i < N_POINTS; i++) {
    f = feature(adc_val[i]);
    if (f < f_lo) f_lo = f;
    if (f > f_hi) f_hi = f;
  }
  f_mid = (f_hi + f_lo) / 2.0;
  f_half = (f_hi - f_lo) / 2.0;
  if (f_half < 1e-3) {
    Serial.println(F("All ADC values are the same. Check the sensor wiring and measure again."));
    fitted = 0;
    return;
  }
  for (i = 0; i < N_POINTS; i++)
    t[i] = (feature(adc_val[i]) - f_mid) / f_half;

  if (!least_squares(t, dist_mm, N_POINTS, degree + 1)) {
    Serial.println(F("Fitting failed: ADC values repeat too much for this degree. Lower the degree."));
    fitted = 0;
    return;
  }
  fitted = 1;

  mean = 0.0;
  for (i = 0; i < N_POINTS; i++)
    mean += dist_mm[i];
  mean /= N_POINTS;

  Serial.println();
  Serial.println(F("  dist(mm)     ADC   fit(mm)   err(mm)"));
  ss_res = ss_tot = 0.0;
  for (i = 0; i < N_POINTS; i++) {
    f = eval_t(t[i]);
    err = f - dist_mm[i];
    ss_res += err * err;
    ss_tot += (dist_mm[i] - mean) * (dist_mm[i] - mean);
    print_col(dist_mm[i], 10, 0);
    print_col(adc_val[i], 8, 1);
    print_col(f, 10, 1);
    print_col(err, 10, 1);
    Serial.println();
  }
  r2 = 1.0 - ss_res / ss_tot;
  rmse = sqrt(ss_res / N_POINTS);
  Serial.print(F("R^2 = ")); Serial.print(r2, 5);
  Serial.print(F(", RMSE = ")); Serial.print(rmse, 2); Serial.println(F(" mm"));

  check_monotonic();
  print_function(r2, rmse);
  if (method == METHOD_POLY)
    print_expanded();
}

// More ADC means a closer ball, so the curve must go down all the way.
// A wiggle between the measured points is the usual sign of overfitting.
void check_monotonic()
{
  float x_lo = adc_val[0], x_hi = adc_val[0], prev, cur;
  int i, bad = 0;

  for (i = 1; i < N_POINTS; i++) {
    if (adc_val[i] < x_lo) x_lo = adc_val[i];
    if (adc_val[i] > x_hi) x_hi = adc_val[i];
  }
  prev = eval_fit(x_lo);
  for (i = 1; i <= 100; i++) {
    cur = eval_fit(x_lo + (x_hi - x_lo) * i / 100.0);
    if (cur > prev + 0.5)
      bad = 1;
    prev = cur;
  }
  if (bad)
    Serial.println(F("warning: the curve goes up somewhere between the points (overfitting?). Try a lower degree."));
}

// Ready-to-paste C function. Horner form on the normalized t keeps float precision.
void print_function(float r2, float rmse)
{
  int k;

  Serial.println();
  Serial.println(F("//////// copy from here ////////"));
  Serial.print(F("// curve_fitting: "));
  Serial.print(method == METHOD_POLY ? F("polynomial of ADC") : F("polynomial of 1000/ADC"));
  Serial.print(F(", degree ")); Serial.print(degree);
  Serial.print(F(", R^2 = ")); Serial.print(r2, 5);
  Serial.print(F(", RMSE = ")); Serial.print(rmse, 2); Serial.println(F(" mm"));
  Serial.println(F("float volt_to_distance(unsigned int a_value)"));
  Serial.println('{');
  if (method == METHOD_INV) {
    Serial.println(F("  if (a_value < 1) a_value = 1;"));
    Serial.print(F("  float t = (1000.0 / a_value - "));
  } else {
    Serial.print(F("  float t = ((float)a_value - "));
  }
  print_num(f_mid);
  Serial.print(F(") / "));
  print_num(f_half);
  Serial.println(';');

  // c0 + t * (c1 + t * (c2 + t * c3))
  Serial.print(F("  return "));
  print_num(coef[0]);
  for (k = 1; k <= degree; k++) {
    Serial.print(F(" + t * "));
    if (k < degree)
      Serial.print('(');
    print_num(coef[k]);
  }
  for (k = 1; k < degree; k++)
    Serial.print(')');
  Serial.println(';');
  Serial.println('}');
  Serial.println(F("//////// copy to here ////////"));
}

// Same polynomial in plain powers of x, to compare with the spreadsheet trendline.
// a_k = sum_{j>=k} c_j * C(j,k) * (-f_mid)^(j-k) / f_half^j
void print_expanded()
{
  float a[MAX_DEG + 1], term, binom;
  int j, k, p;

  for (k = 0; k <= degree; k++) {
    a[k] = 0.0;
    for (j = k; j <= degree; j++) {
      binom = 1.0;
      for (p = 0; p < k; p++)
        binom = binom * (j - p) / (p + 1);
      term = coef[j] * binom;
      for (p = 0; p < j - k; p++)
        term *= -f_mid;
      for (p = 0; p < j; p++)
        term /= f_half;
      a[k] += term;
    }
  }

  Serial.println();
  Serial.println(F("Expanded (compare with the spreadsheet trendline; use the version above in code):"));
  Serial.print(F("  y = "));
  for (k = degree; k >= 0; k--) {
    print_num(a[k]);
    if (k >= 1) Serial.print('x');
    if (k >= 2) { Serial.print('^'); Serial.print(k); }
    if (k > 0) Serial.print(F(" + "));
  }
  Serial.println();
}

// ---------------------------------------------------------------- test mode

void test_mode()
{
  unsigned int spread;
  float x;

  if (!fitted) {
    Serial.println(F("no fitted curve yet"));
    return;
  }
  Serial.println(F("Test. Enter: measure,  q: back to menu"));
  while (1) {
    read_line();
    if (line[0] == 'q')
      return;
    x = measure_adc(3, &spread);
    Serial.print(F("  ADC = ")); Serial.print(x, 1);
    Serial.print(F("  ->  distance = ")); Serial.print(eval_fit(x), 1);
    Serial.println(F(" mm"));
  }
}

// ---------------------------------------------------------------- printing helpers

// Print a C float literal with 7 significant digits, e.g. -1.234567E-03.
void print_num(float v)
{
  int e;
  float m;

  if (v == 0.0) {
    Serial.print(F("0.0"));
    return;
  }
  if (v < 0) {
    Serial.print('-');
    v = -v;
  }
  e = (int)floor(log10(v));
  m = v / pow(10.0, e);
  if (m < 1.0) { m *= 10.0; e--; }
  if (m >= 9.9999995) { m /= 10.0; e++; }   // would round up to "10.000000"
  Serial.print(m, 6);
  Serial.print('E');
  Serial.print(e < 0 ? '-' : '+');
  if (e < 0) e = -e;
  if (e < 10) Serial.print('0');
  Serial.print(e);
}

void print_col(float v, int width, int digits)
{
  char buf[16];
  dtostrf(v, width, digits, buf);
  Serial.print(buf);
}

// ---------------------------------------------------------------- IR median filter (from 20_example_1)

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
  // verbose: 0 - normal operation, 1 - observing the internal procedures, and 2 - calculating elapsed time.
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

  for (unsigned int i = 0; i < n; i++) {
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
    for (unsigned int i = 0; i < n; i++) {
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
