#include <SharpIR.h>
#include <Servo.h>
#include <Wire.h>
#include "Kalman.h"
#define model 1080

Servo Kvadrat;
Servo Treugol;

Servo Palka;
Servo Krug;

Servo SputnikServo0;
Servo SputnikServo1;
Servo SputnikServo2;
Servo SputnikServo3;

Servo Rudder_on_kvadrat;
Servo Rudder_on_treugol;
Servo Rudder_on_palka;
Servo Rudder_on_krug;

Servo Rudder_kvadrat;
Servo Rudder_treugol;
Servo Rudder_palka;
Servo Rudder_krug;

int IA1 = 4;
int IA2 = 3;

int dalnomerPin = A0;
SharpIR SharpIR(dalnomerPin, model);
int distance=100;

int end_verh_pin= 30;
int end_niz_pin = 25;
int end_verh =0;
int end_niz =0;

int sputnik_button_pin=41;
int sputnik_button;

int first_sputnik_pin=50;
int second_sputnik_pin=51;
int thrid_sputnik_pin=52;

int sputkik_go_flag=0;
int sputnik_old_time=0;
int sputnik_time=0;

int sputnik_go_flag=0;

int rudder_go_flag=0;
int rudder_button_pin=40;

int Treu_angle = 0;
int Palka_angle = 0;
int Krug_angle = 0;
int Kvadrat_angle = 0;



Kalman kalmanX;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
/* IMU Data */
int var;
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle; // Angle calculate using the accelerometer
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
uint32_t timer;

int AngleX=0;
int AngleY=0;


void setup() {
	Wire.begin();
	Serial.begin(9600);
	i2cWrite(0x6B,0x00); // Disable sleep mode 
	kalmanX.setAngle(180); // Set starting angle
	kalmanY.setAngle(180);
	timer = micros();
	
	var=0;
	sputnik_go_flag=0;

	pinModing();

	digitalWrite(first_sputnik_pin, LOW);
	digitalWrite(second_sputnik_pin, LOW);
	digitalWrite(thrid_sputnik_pin, LOW);
	
	rudder_off();

}

void loop() {
	
	Raschet();

	AngleX= int(kalAngleX)/2;
	AngleY= int(kalAngleY)/2;
	
	Printing();

	Vector_of_throttle();

	Sharping_and_pining();

	if (digitalRead(sputnik_button_pin)==1){ 
		sputnik_go_flag=1;	
		sputnik_old_time =millis(); 	
	}
	if (sputnik_go_flag==1) 	detach_sputnik();

	if (digitalRead(rudder_button_pin)==1) rudder_go_flag=1;
	if (digitalRead(end_niz_pin) == 1) rudder_go_flag=0;

	if (rudder_go_flag == 1) rudder_on();
	else rudder_off();

	//	rudder_on();



}



void i2cWrite(uint8_t registerAddress, uint8_t data)
{
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.write(data);
	Wire.endTransmission(); // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) 
{
	uint8_t data[nbytes];
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.endTransmission(false); // Don't release the bus
	Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
	for(uint8_t i = 0; i < nbytes; i++)
	data [i]= Wire.read();
	return data;
}

void Raschet()
{
		while (var<100)
	{
			/* Update all the values */
		uint8_t* data = i2cRead(0x3B,14);
		accX = ((data[0] << 8) | data[1]);
		accY = ((data[2] << 8) | data[3]);
		accZ = ((data[4] << 8) | data[5]);
		tempRaw = ((data[6] << 8) | data[7]);
		gyroX = ((data[8] << 8) | data[9]);
		gyroY = ((data[10] << 8) | data[11]);
		gyroZ = ((data[12] << 8) | data[13]);
		/* Calculate the angls based on the different sensors and algorithm */
		accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
		accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG; 
		double gyroXrate = (double)gyroX/131.0;
		double gyroYrate = -((double)gyroY/131.0);
		gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/10000); // Calculate gyro angle using the unbiased rate
		gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/10000);
		kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/10000); // Calculate the angle using a Kalman filter
		kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/10000);
		timer = micros();
		var++;
	}

	uint8_t* data = i2cRead(0x3B,14);
	accX = ((data[0] << 8) | data[1]);
	accY = ((data[2] << 8) | data[3]);
	accZ = ((data[4] << 8) | data[5]);
	tempRaw = ((data[6] << 8) | data[7]);
	gyroX = ((data[8] << 8) | data[9]);
	gyroY = ((data[10] << 8) | data[11]);
	gyroZ = ((data[12] << 8) | data[13]);
	/* Calculate the angls based on the different sensors and algorithm */
	accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
	accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG; 
	double gyroXrate = (double)gyroX/131.0;
	double gyroYrate = -((double)gyroY/131.0);
	gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
	gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
	kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
	kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
	timer = micros();
	delay(1);

}

void pinModing()
{

	Kvadrat.attach(5);
	Treugol.attach(6);

	Palka.attach(7);
	Krug.attach(8);

	Rudder_on_kvadrat.attach(9);
	Rudder_on_treugol.attach(10);
	Rudder_on_palka.attach(11);
	Rudder_on_krug.attach(12);

	Rudder_kvadrat.attach(17);
	Rudder_treugol.attach(14);
	Rudder_palka.attach(15);
	Rudder_krug.attach(42);


	SputnikServo0.attach(34);
	SputnikServo1.attach(35);
	SputnikServo2.attach(36);
	SputnikServo3.attach(37);

	SputnikServo0.write(10); // Close servo door
	SputnikServo1.write(90); // Close sputnik	
	SputnikServo2.write(90); // Close sputnik
	SputnikServo3.write(90); // Close sputnik

	pinMode(end_niz_pin, INPUT);
	pinMode(end_verh_pin, INPUT);
	pinMode(sputnik_button_pin, INPUT);

	pinMode(first_sputnik_pin, OUTPUT);
	pinMode(second_sputnik_pin, OUTPUT);
	pinMode(thrid_sputnik_pin, OUTPUT);

	pinMode(IA1, OUTPUT);
  	pinMode(IA2, OUTPUT);


}


void Vector_of_throttle()
{

	// Kvadrat.write(80); //-
	// Treugol.write(90); //-
	
	// Palka.write(95); //-
	// Krug.write(95); //-
	if (AngleX<90)
	{ 
		Treu_angle =98+map(AngleX, 90, 105, 0, 40);
		Treugol.write(Treu_angle); 

	}
	else 
	{
		Palka_angle =103+map(AngleX, 90, 75, 0, 40);
		Palka.write(Palka_angle); //Palka
	}
	
	if (AngleY<90)	
	{ 
		Krug_angle=108+map(AngleY, 90, 105, 0, 40);
		Krug.write(Krug_angle); //Krug
	}
	
	else 
	{
		Kvadrat_angle=90+map(AngleY, 90, 75, 0, 40);
		Kvadrat.write(Kvadrat_angle); 
	}


}


void Sharping_and_pining()
{
	distance=SharpIR.distance();  // this returns the distance to the object you're measuring
	
	end_niz=digitalRead(end_niz_pin); 
	end_verh=digitalRead(end_verh_pin);

	if ((distance < 50 | distance > 150) and end_niz==0 ) {
		analogWrite (IA1, 150); //вниз
      	analogWrite (IA2, LOW);	
      	Serial.println("val1");
	}
	else if (end_niz == 1)
	{
		analogWrite (IA1, LOW); // стоп
      	analogWrite (IA2, LOW);
		Serial.println("val2");
	}


	if (distance > 50 and end_verh==0 and distance < 80) {
		analogWrite (IA1, LOW); // вверх
      	analogWrite (IA2, 150);
		Serial.println("val3");

	}	
	else if (end_verh == 1 and distance > 50 and distance < 81)
	{
		analogWrite (IA1, LOW); // стоп
      	analogWrite (IA2, LOW);
		Serial.println("val4");
	}

}

void detach_sputnik()
{

	sputnik_time=millis(); 

	SputnikServo0.write(70); // Open servo door

	if (sputnik_time - sputnik_old_time > 2000) 
	{
		SputnikServo1.write(70);
	} 
	
	if (sputnik_time - sputnik_old_time > 5000) 
	{
		SputnikServo2.write(70);
	} 
	
	if (sputnik_time - sputnik_old_time > 7000) 
	{
		SputnikServo3.write(70);
	} 
					
	if (sputnik_time - sputnik_old_time > 9000)
	{
	SputnikServo0.write(10); // Close door
	sputnik_go_flag=0;
	}
 	
}
void rudder_on()
{
	Serial.println("Rudder_on");

	Rudder_on_kvadrat.write(0);
	Rudder_on_treugol.write(3);
	Rudder_on_palka.write(15);
	Rudder_on_krug.write(15);

	Rudder_treugol.write((-AngleX - AngleY + 180)*3 + 90);
	Rudder_krug.write((AngleX - AngleY)*3 + 90);	
	Rudder_palka.write(( AngleX+AngleY-180)*3+90);
	Rudder_kvadrat.write((-AngleX+AngleY)*3 + 90);
	// Rudder_kvadrat.write(120);

	// Rudder_palka.write(90);
}



void Printing()
{
	Serial.print("X:");
	Serial.print(AngleX);
	Serial.print(" Y:");
	Serial.print(AngleY);


	Serial.print("  Verh: ");
	Serial.print(end_verh);
	Serial.print("  Niz: ");
	Serial.print(end_niz);
	
	Serial.print("  Sharp: ");
	Serial.print(distance);

	Serial.print("  Treugol: ");
	Serial.print(Treu_angle);

	Serial.print("  Palka: ");
	Serial.print(Palka_angle);

	Serial.print("  Krug: ");
	Serial.print(Krug_angle);

	Serial.print("  Kvadrat: ");
	Serial.print(Kvadrat_angle);

	Serial.print("    ");
	Serial.print((AngleX - AngleY)*3);

	Serial.println();
}


void rudder_off()
{

	Rudder_treugol.write(90);
	Rudder_krug.write(90);	
	Rudder_palka.write(90);
	Rudder_kvadrat.write(90);

	Rudder_on_kvadrat.write(85);
	Rudder_on_treugol.write(88);
	Rudder_on_palka.write(95);
	Rudder_on_krug.write(98);
}