#define PIN_LED 7
unsigned int toggle;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial)
  {
    ;
  }
  toggle == 0;
  digitalWrite(PIN_LED, toggle);
}

void loop()
{
  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(200);
}

int toggle_state(int toggle)
{
  return !toggle;
}
