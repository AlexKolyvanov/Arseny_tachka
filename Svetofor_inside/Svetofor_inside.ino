//Red, RedYellow, Green, BlinkingGreen, Yellow -> 0, 1, 2, 3, 4

const int stages = 5;
const int transmitInterval = 33, blinkInterval = 500;
const byte voltagePin = A0, redLed = A3, yellowLed = A4, greenLed = A5;

unsigned long curTime = 0, lastChangeTime = 0, lastTransmitTime = 0, lastBlinkTime = 0;
const int stageIntervals[stages] = {7000, 2000, 4000, 3000, 2000}; //Red, RedYellow, Green, BlinkingGreen, Yellow;
const byte stageCmds[stages] =     {0, 1, 2, 3, 4};

byte curStage = 0;

boolean greenOn = false;
byte inByte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(voltagePin, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  // delay(3000);
  curTime = millis();
}

void loop() {
  for(int stage=0; stage<stages; stage++){
    while(1) {
      curTime = millis();
      transmit(stageCmds[stage]);
      lightsUp(stageCmds[stage]);
      if ((curTime - lastChangeTime) > stageIntervals[stage]) {
        lastChangeTime = curTime;
        break;
      }
    }
  }
}

void transmit(byte transCmd) {
  if((curTime - lastTransmitTime) > transmitInterval) {
    Serial.write(transCmd);
    lastTransmitTime = curTime;
  }
}

void lightsUp(byte curStage) {
  switch (curStage) {
    case 0: //Red
      digitalWrite(redLed, HIGH);
      digitalWrite(yellowLed, LOW);
      digitalWrite(greenLed, LOW);
      greenOn = false;
      break;
    case 1: //RedYellow
      digitalWrite(redLed, HIGH);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(greenLed, LOW);
      greenOn = false;
      // do something
      break;
    case 2: //Green
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(greenLed, HIGH);
      greenOn = true;
      // do something
      break;
    case 3: //Blinking Green
      // Serial.println("blink");
      if((curTime - lastBlinkTime) > blinkInterval) {
        // Serial.println(curTime);
        digitalWrite(redLed, LOW);
        digitalWrite(yellowLed, LOW);
        if(greenOn) {
          digitalWrite(greenLed, LOW);
          greenOn = false;
        }
        else {
          digitalWrite(greenLed, HIGH);
          greenOn = true;
        }
        lastBlinkTime = curTime;
      }
      break;
    case 4: //Yellow
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(greenLed, LOW);
      greenOn = false;
      // do something
      break;
  }
}