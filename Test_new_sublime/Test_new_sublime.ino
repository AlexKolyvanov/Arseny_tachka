//Test_new_sublime.ino

int svetSignal=0;
int time=0;
int oldTime=0;

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
}

void loop() {
	SvetoforTest();
 }

void SvetoforTest() 
{
  time=millis();		
  if (Serial1.available()) 
  {
  	svetSignal = Serial1.read(); 
  	oldTime = time;
  }
  else if (time-oldTime>1500)
  {
  	svetSignal = 6;
  }

  Serial.println(svetSignal, DEC);
}


