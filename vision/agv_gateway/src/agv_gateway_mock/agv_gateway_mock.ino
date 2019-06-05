void setup()
{
  while (!Serial);
  Serial.begin(115200);
}

void loop()
{
  Serial.println("#S#0.23434");
  delay(5000);
}
