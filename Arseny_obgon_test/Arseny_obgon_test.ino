//Arseny_tachka_v2.ino
#include <Servo.h>
#include <SharpIR.h>
//3.12.17.ino
SharpIR SharpIR(A0, 1080);

static const byte PACKET_SIZE = 128;
static const byte VALUE_SIZE = 1;
static const boolean SEPARATE_VALUES = false;

const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
int CLKpin = 20; // CLK - Clock 
int SIpin = 21; // SI - Serial input 
int AOpin = A15; // AO - Analog output
int IntArray[128];
char ptxt[128 + 1];
int SumAngle=0;
int knopp;
int TurnCoef[128];
int error=0;
int old_error=0;
int ActiveSens=0;
int svetSignal=6;
int time=0;
int oldTime=0;
int step=0;

Servo myservo;
Servo dvig;

void setup() {

  for(int i=0; i<128; i++){
    TurnCoef[i]=-64+i;
  }

  Serial.begin(115200);
  Serial1.begin(115200);
  
  CameraPreparation();

  myservo.attach(7);
  dvig.attach(8);
  dvig.writeMicroseconds(1500) ;
  myservo.write(82) ;

  delay(1000);
}

void loop()
{

  if (step == 0)  // Ждем светофор
  {
    SvetoforRead();
    if (svetSignal == 2)
    {
      step=1;
    }
  }  


  if (step == 1 ) // едем по линии до тачки
  {

    Reading_line();

    int error = (SumAngle/ActiveSens);
    myservo.writeMicroseconds (1420-(error*9)+1.10051105*(error-old_error));
    int old_error = error;
    dvig.writeMicroseconds(1604); // Рабочая строка

    Printing();

    int dis=SharpIR.distance();
    if (dis <= 50)
    {
      step = 2;
    }
  }
  if (step == 2) // сворачиваем налево
  {
    myservo.writeMicroseconds (1320); // Поворот налево 
    dvig.writeMicroseconds(1604);     // Газуем  
    delay(1000);                      // И так одну секунду
    step = 3 ;
    
  }

  if (step == 3) // Выравниваемся и едем до встечки
  {

    Reading_line();

    int error = (SumAngle/ActiveSens);
    myservo.writeMicroseconds (1420-(error*9)+1.10051105*(error-old_error));
    int old_error = error;
    // dvig.writeMicroseconds(1604); // Рабочая строка
 
    dvig.writeMicroseconds(1604);       
    Printing();

    int dis=SharpIR.distance();
    if (dis <= 50)
    {
      step = 4;
    }
  }
  
  if (step == 4) 
  {
  	myservo.writeMicroseconds (1500); // Поворот направо 
    dvig.writeMicroseconds(1604);     // Газуем  
    delay(1000);                      // И так одну секунду
    step = 5 ;
  }

  if (step == 5)
  {

  	Reading_line();

    int error = (SumAngle/ActiveSens);
    myservo.writeMicroseconds (1420-(error*9)+1.10051105*(error-old_error));
    int old_error = error;
    // dvig.writeMicroseconds(1604); // Рабочая строка
 
    dvig.writeMicroseconds(1604);       
    Printing();

    if (ActiveSens > 60)
    {
      step = 6;
    }	
    
    if (step == 6 )
    {
    	dvig.writeMicroseconds(1500);
    }

  }

    
}


void ClockPulse(){
  delayMicroseconds(1);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(CLKpin, LOW);
}



void SvetoforRead() 
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
  while(Serial1.available()){Serial1.read();}
}


void Printing()
{

  Serial.print(SumAngle);
  Serial.print(" - ");
  Serial.print(ActiveSens);
  Serial.print(" --- ");
  Serial.print(" Err:");
  Serial.print(error);
  Serial.print(", Svet = ");
  Serial.println(svetSignal);
}

// void CalculationSpeed()
// {
//   int dis=SharpIR.distance();
//   if(dis<=50)
//   {
//     dvig.writeMicroseconds(1300);
//   }
//   else 
//   {
//     if(svetSignal==0|svetSignal==1|svetSignal==5)
//     {
//       if(ActiveSens>=30){
//         dvig.writeMicroseconds(1300);
        
//       } 
//       else{
//         if(ActiveSens<=5){
//           dvig.writeMicroseconds(1500);
//           delay(500);
//           dvig.writeMicroseconds(1200);
//           delay(1000);
//         }
//         dvig.writeMicroseconds((1620-error/10)-25);

//       }
//     }
//     else
//     {
//       dvig.writeMicroseconds(1620-error/10);
//     } 
//   }
// }

void CameraPreparation()
{
  pinMode(CLKpin, OUTPUT); 
  pinMode(SIpin, OUTPUT);

  ADCSRA &= ~PS_128; 
  ADCSRA |= PS_32; // теперь одно АЦП преобразование займет ~30 мкс

  analogReference(DEFAULT);

    digitalWrite(21,LOW);
    digitalWrite(20,LOW);
  
    digitalWrite(SIpin, HIGH);
    ClockPulse(); 
    digitalWrite(SIpin, LOW);

    for(int i=0;i< 260;i++)
    {
      ClockPulse(); 
    }
}


void Reading_line()
{

  ActiveSens=0;
  SumAngle=0;

  // Запуск нового измерения
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);

    // Чистка регистра датчика от мусора
    for(int i = 0; i < 128; i++){
      ClockPulse();     
    }

    // Экспозиция 3мс
    delay(60);

    // Запуск сбора данных
    digitalWrite(SIpin, HIGH);
    ClockPulse();
    digitalWrite(SIpin, LOW);

    // Чтение 128 пикселей
    for(int i=0; i < 128; i++){
        delayMicroseconds(20); // пауза для фиксации значения на АЦП
        IntArray[i] = analogRead(AOpin);
        ClockPulse(); 
    }

    Serial.print("|");  
    for(int i=10; i<118; i++){
        if (IntArray[i] > 860)
        {
          // Serial.print("1");
        }
        else
        {
          ActiveSens++;
          // Serial.print(" ");
          SumAngle+=TurnCoef[i];
        }
      }
      Serial.print("|");
}


