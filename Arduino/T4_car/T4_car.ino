#include <Wire.h>
#include <Servo.h>
#include <SharpIR.h>
#include <Smartcar.h>
//#include <QTRSensors.h>
#include <RcCar.h>
//#include <math.h>

#define ultrasonic_Front_Center 112
#define ultrasonic_Front_Side 115

//Ir Sensors
#define frontRightIrPin A3
#define backRightIrPin A4
#define backIrPin A5


//Led lights
#define led_lights 9

//Odometer
#define odometer 3

#define MAXFORWSPEED 150
#define MAXREVSPEED 100
#define rcController 4
#define speedPin 2
//esc
Servo esc;
Servo steering;
Odometer encoder;
RcCar* car ;
//char command;



float angle=90;
float speeds=0;

// For speed
unsigned long lastDist=0; // measures the previous movements
unsigned long lastTime=0; //  and stores them globally just to get the remainder which is the last distance and time for that traveled


SharpIR frontRightIrSensor(GP2YA41SK0F,frontRightIrPin);
SharpIR backRightIrSensor(GP2YA41SK0F,backRightIrPin );
SharpIR backIrSensor(GP2YA41SK0F,backIrPin);
//QTRSensorsRC((unsigned char[]){6}, 1, 4, QTR_NO_EMITTER_PIN);
//QTRSensorsRC qtra((unsigned char[]) {6},1, 100, 255);
//QTRSensorsAnalog qtra((unsigned char[]) {6}, 1, 2000, QTR_NO_EMITTER_PIN);
//unsigned int sensorValues[1];


/*
reads a byte from serial port, current speed and angle , reads them as bytes and tranforms them into floats. used when you get info from the proxy
*/

void byteDecode(char byteIn, float *speed, float * angle) {
  float tempAng = *angle;
  float tempSpd = *speed;
  *angle = ((byteIn & 31) - 15) * 4;
  //Serial2.println("qwdwe  "+String(((float)((byteIn & 224) >> 5)-3)));
   //Serial2.println("abba  "+String(((float)((byteIn & 224) >> 5)-3)/2));
  *speed = ((float)((byteIn & 224) >> 5) - 3) / 2;

  if (*angle == 48) {
    //command='a';
    *angle = tempAng;
  }
}


/*
you set how many times you want to get readings from a certain sensor, adds them as a sum then averages them out
*/
int readSensor(int readings,SharpIR sensor){
	//int arr[readings];
	int x=0;
	for(int i=0;i<readings;i++){
		x+= sensor.getDistance();
	}
	return x/readings;
}



void setup() {
  //	command='c';
	speeds = 0; //speed of vehicle
	angle = 90; //steering angle
	//Inputs and outputs
	pinMode(led_lights, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(odometer, INPUT);
	pinMode(rcController, INPUT);
	pinMode(speedPin, INPUT);


  //Servos
	esc.attach(10);
	steering.attach(5);
	esc.write(1380);

	encoder.attach(odometer);

	car = new RcCar(MAXFORWSPEED * 10, 1400, MAXREVSPEED * 10, 1100, 1380, esc, steering);
  //esc.writeMicroseconds(50);

	Wire.begin(); // I2C comms
	encoder.begin();
	//Serial
	Serial2.begin(9600);
	Serial.begin(9600);
	//Serial.println("g");
	car->setSpeed(0);
	delay(500);

	/* for (int i = 0; i < 100; i++){
    	qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  	}*/
}




/*
reads from the integrated  circuit that returns an int from master and has all info on slaves */
int readI2C(int address) {
	//Serial.println(String(Wire.requestFrom(address,1)));
	Wire.requestFrom(address, 1);
	char x = (char)0;
	while (Wire.available()) {
    	x = Wire.read();
		if (x > 0) {
			//Serial.println("pinged "+String((int)x));
			return x;
		};

	}
	return x;

}

//Ultrasonic controllers
unsigned long tick = 0; //measuring how often you use the US
boolean ping = false;
char sensorId = 0;


char I2C[2] = {ultrasonic_Front_Center, ultrasonic_Front_Side};
//data storage
char SensorData[7] = {0, 0, 0, 0, 0, 0, 0};


void loop() {
		//tick=millis();

		Wire.beginTransmission(I2C[sensorId]);
		Wire.write(0x02); // setthe measuring distance in the register two then puts a value to the register
		Wire.write(14); //
		Wire.endTransmission(1);
	
		Wire.beginTransmission(I2C[sensorId]);
		Wire.write(0x00); //
		Wire.write(0x51); //
		Wire.endTransmission(1);
		

		unsigned long times = millis();

		int n=0; //data average of a reading set from frontIRsensor, save it in the array, then read again from backRightIrSensorIRsensor then a last time for backIRsensor
		n=readSensor(25,frontRightIrSensor);
		char distanceFrontRightIr = ((n < 20 ? (n) : 0) / 2 & 31) | (5 << 5); //Calculate the distance in centimeters and store the value in a variable
		//distanceFrontRightIr |=	5 << 5;
		SensorData[5] = distanceFrontRightIr;

	 	n=readSensor(25,backRightIrSensor);
		char distanceBackRightIr = ((n<20 ? (n) : 0) / 2 & 31) | (1 << 5);
		//distanceBackRightIr |=	1 << 5;
		SensorData[1] = distanceBackRightIr;

		n=readSensor(25,backIrSensor);
		char distanceBackIr = ((n < 20 ? ( n) : 0) / 2 & 31) | (4 << 5);
		//distanceBackIr|=(4 << 5);
		//distanceBackRightIr |=	(4 << 5);
		SensorData[4] = distanceBackIr;

    // Read once approx every 500ms
    // read the distance traveled every 500ms
    if(times - lastTime > 2000){
		unsigned long dist = encoder.getDistance()*2;
		//Serial2.println("dist "+String(encoder.getDistance()));
		int spd = ((dist - lastDist) / ((double)(times - lastTime) / 1000));
		//Serial2.println("delta dist "+String(dist - lastDist)+" delta time "+ String((times - lastTime)));
		Serial2.println(spd);
		// Get speed since last iteration
		lastDist = dist;
		lastTime = times;
		spd |= 6 << 5;
		SensorData[5] = spd & 31;
    }


	/*if (ping) {
		Wire.beginTransmission(I2C[sensorId]);
		Wire.write(0x02); // setthe measuring distance in the register two then puts a value to the register
		Wire.write(14); //
		Wire.endTransmission(1);
		//delay(20);

		//delay(20);
		Wire.beginTransmission(I2C[sensorId]);
		Wire.write(0x00); //
		Wire.write(0x51); //
		Wire.endTransmission(1);
		tick = millis();
		ping = false;
		tick = millis();
	}*/

/* The below changes them in order to read from them */
	/*if (!ping && millis() - tick > 5) {
		Wire.beginTransmission(I2C[sensorId]);
		Wire.write(0x03);
		Wire.endTransmission(1);
		char v = readI2C(I2C[sensorId]);
	 
		SensorData[2 + sensorId] = ((v <= 50 && v != 0 ? v : 0) / 2 & 31) | ((2+sensorId)<<5);
		
		ping = true;
		sensorId = 0;//(sensorId + 1) % 2;
	}*/

	for (int i = 1; i < sizeof(SensorData); i++) {

		Serial.print(SensorData[i]);
		//if(i==2 && ((SensorData[i]>>5)&7)==2)
		//Serial.println("ultras sees "+String(((SensorData[2])&31)*2)+" id is "+String((SensorData[2]>>5)&7));
		
		//Serial2.println(String((SensorData[i]&31)*2)+" id "+String((SensorData[i]>>5)&7));
	}



	Wire.beginTransmission(I2C[sensorId]);
	Wire.write(0x03);
	Wire.endTransmission(1);
	char v = readI2C(I2C[sensorId]);
	SensorData[2 + sensorId] = ((v <= 50 && v != 0 ? v : 0) / 2 & 31) | ((2+sensorId)<<5);
	sensorId = 0;//(sensorId + 1) % 2;


/* controller stuff here */
	unsigned int steeringCH = pulseIn(rcController, HIGH);
		
	unsigned int speedCH = pulseIn(speedPin, HIGH);


/* reading from proxy and bluetooth feedback */
	if (Serial.available()) {
		char k = Serial.read();
		if(k!=31){
			byteDecode(k, &speeds, &angle);
			//Serial2.println("inputs is "+String((int)k));
			//Serial2.println("angle is"+String(angle));
			//Serial2.println("speed is"+String(speeds));
			car->setAngle(angle + 120);
		}
	}else {
		//Serial2.println("not available####3");

	}

/*filter out input from rc controller*/
/*set speed from rc controller*/
	if (speedCH / 10 < 190) {
		//Serial2.println(speedLimit(speedCH));
		if( car->isValidMoveSpeed(speedLimit(speedCH))){
			ping=true;
			//car->setSpeed(0);
		}
		car->setRawSpeed(&speedLimit, speedCH);
		Serial2.println(ping);
		if(ping)
			car->setRawSpeed(&speedLimit, speedCH);
		else{
			car->setSpeed(2);	
		}
	}else {
		speeds=0;
		car->setSpeed(0);
		ping=false;
		car->setAngle(90);
	}
Serial2.println(" delay is"+String(millis()-times));
}
/* proxy input is : -60 to 60 for steering angle, 5 bits of which are steering info 3 bits are ms. the frist three bytes should show the speed */



/*set global speed limit for safety*/
int speedLimit(int speed) {
	if (speed / 10 < 140 && speed / 10 > 130) { //if in neutral
		return 1387;
	}else if (speed / 10 > MAXFORWSPEED) { //max speed forward
		return MAXFORWSPEED * 10;
	}else if (speed / 10 < MAXREVSPEED) { //max spedd in reverse

		return MAXREVSPEED * 10;
	}else {
		return speed;
	}
}


bool isFloat(char c) {
	return (c >= 48 && c <= 57) || c == 46;
}

bool isNumber(char c) {
	return c >= 48 && c <= 57;
}

bool isLetter(char c) {
	return c >= 92 && c <= 122;

}
