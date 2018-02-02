int val;
int LED = 13;
int time=0;
int oldTime=0;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}
void loop()
{
  if (Serial1.available())
  {
    val = Serial1.read();
    // При символе "1" включаем светодиод
    if (val == '1')
    {
      digitalWrite(LED, HIGH);
    }
    // При символе "0" выключаем светодиод
    if ( val == '0')
    {
      digitalWrite(LED, LOW);
    }
  }
      
	time = millis();
	if (time-oldTime>100)
	{
		Serial1.println(String(analogRead(A15)));	
		oldTime=time;
	}
  
  Serial.println(String(analogRead(A15)));

}