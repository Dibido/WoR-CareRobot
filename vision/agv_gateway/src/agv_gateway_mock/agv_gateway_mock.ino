/*
   AGV gateway mock
   A mocker to mock the data sent from the AGV, used to test the agv_parser
   @Author: Dibran Dokter
*/

void setup()
{
  while (!Serial)
  {
    // Wait for serial
  }
  Serial.begin(115200);
}

void loop()
{
  Serial.println("#S#0.23434");
  delay(5000);
}
