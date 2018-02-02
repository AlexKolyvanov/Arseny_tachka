#include <Servo.h>
//3.12.17.ino

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


Servo myservo;
Servo dvig;

void setup() {

	for(int i=0; i<128; i++){
		TurnCoef[i]=-64+i;
	}

  Serial.begin(115200);

  pinMode(CLKpin, OUTPUT); 
  pinMode(SIpin, OUTPUT);

  myservo.attach(7);
  dvig.attach(8);
  dvig.writeMicroseconds(1500) ;
  myservo.write(82) ;

  // ускоряем АЦП в 4 раза 
  ADCSRA &= ~PS_128; 
  ADCSRA |= PS_32; // теперь одно АЦП преобразование займет ~30 мкс

  analogReference(DEFAULT);

  // Установка всех GPIO в LOW:
 //for( int i=0; i< 14; i++ ){
    //digitalWrite(i, LOW); 
  //} 
digitalWrite(21,LOW);
digitalWrite(20,LOW);
  // Запуск первого измерения
  digitalWrite(SIpin, HIGH);
  ClockPulse(); 
  digitalWrite(SIpin, LOW);
 
  // Пока идет измерение, чистим содержимое регистра датчика
  for(int i=0;i< 260;i++){
    ClockPulse(); 
 knopp=digitalRead(14);//
 if (knopp==1){
  }
 else {delay(100);
 knopp=digitalRead(14); }
  }
	// for(int i=0; i<128; i++)
	// {
 //    	AngleTurn[i]=-64+i;
	// }

  delay(1000);
}

void loop(){
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

  // Отправка вектора значений через UAR
  Serial.print("|");	
  for(int i=10; i<118; i++){
  	//Serial.print(IntArray[i]/100);
  	// Serial.print(" ");
  	if (IntArray[i] > 860)
  	{
  		Serial.print("1");

  	}
  	else
  	{
  		
  		ActiveSens++;
  		Serial.print(" ");
  		SumAngle+=TurnCoef[i];

  	}
  }
  Serial.print("|");
   int error = (SumAngle/ActiveSens);
  Serial.print(SumAngle);
  Serial.print(" - ");
  Serial.print(ActiveSens);
  Serial.print(" --- ");
  Serial.print("Err:");
  Serial.println(error);
  myservo.writeMicroseconds (1420-(error*9)+1*(error-old_error));
   int old_error = error;
 // dvig.writeMicroseconds(1604); // Рабочая строка
  dvig.writeMicroseconds(1620-error/10);
  //dvig.write(110) ; 
  // rd.sendPacket();

} 

// Функция, генерирующая синхроимпульс
void ClockPulse(){
  delayMicroseconds(1);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(CLKpin, LOW);
}
