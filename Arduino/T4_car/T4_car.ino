#include <Wire.h>
#include <Servo.h>
#include <SharpIR.h>
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
#define odometer 6

#define MAXFORWSPEED 150
#define MAXREVSPEED 100
#define rcController 4
#define speedPin 2
//esc
Servo esc;
Servo steering;
RcCar* car ;
//char command;



float value;
float speeds;



SharpIR frontRightIrSensor(GP2YA41SK0F,frontRightIrPin);
SharpIR backRightIrSensor(GP2YA41SK0F,backRightIrPin );
SharpIR backIrSensor(GP2YA41SK0F,backIrPin);
//QTRSensorsRC((unsigned char[]){6}, 1, 4, QTR_NO_EMITTER_PIN);
//QTRSensorsRC qtra((unsigned char[]) {6},1, 100, 255); 
//QTRSensorsAnalog qtra((unsigned char[]) {6}, 1, 2000, QTR_NO_EMITTER_PIN);
//unsigned int sensorValues[1];




void byteDecode(int byteIn, float *speed, float * angle) {
  float tempAng = *angle;
  *angle = ((byteIn & 31) - 15) * 4;
  *speed = (((byteIn & 224) >> 5) - 3) / 2;

  if (*angle == 48) {
    //command='a';
    *angle = tempAng;

  }



}

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
  speeds = 0;
  value = 90;
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

  car = new RcCar(MAXFORWSPEED * 10, 1400, MAXREVSPEED * 10, 1100, 1380, esc, steering);
  //esc.writeMicroseconds(50);

  Wire.begin();
  //Serial
  Serial2.begin(9600);
  Serial.begin(9600);
   //Serial.println("g");

	/* for (int i = 0; i < 100; i++){
    	qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  	}*/


}





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
unsigned long tick = 0;
boolean ping = true;
char sensorId = 0;


char I2C[2] = {ultrasonic_Front_Center, ultrasonic_Front_Side};
//data storage
char SensorData[6] = {'0', '0', '0', '0', '0','0'};


void loop() {		

unsigned long times = millis();
	
		int n=0;
	 	n=readSensor(50,frontRightIrSensor);
	 	char distanceFrontRightIr = (n < 20 ? (n) : 0) / 2 & 31; //Calculate the distance in centimeters and store the value in a variable
	 	distanceFrontRightIr |=	5 << 5;
	 	SensorData[5] = distanceFrontRightIr;
	
		n=readSensor(50,backRightIrSensor);
	 	char distanceBackRightIr = ( 20 > n ? (n) : 0) / 2 & 31;
	 	distanceBackRightIr |=	1 << 5;
	 	SensorData[1] = distanceBackRightIr;
	
		n=readSensor(50,backIrSensor);
	 	char distanceBackIr = ((n < 20 ? ( n) : 0) / 2 & 31) | (4 << 5);
	 	//distanceBackIr|=(4 << 5);
	 	//distanceBackRightIr |=	(4 << 5);
	 	SensorData[4] = distanceBackIr;



 	if (ping) {
	    Wire.beginTransmission(I2C[sensorId]);
	    Wire.write(0x02);
	    Wire.write(14);
	    Wire.endTransmission(1);
	    //delay(20);
	   
	    //delay(20);
	    Wire.beginTransmission(I2C[sensorId]);
	    Wire.write(0x00);
	    Wire.write(0x51);;
	    Wire.endTransmission(1);
	    tick = millis();
	    ping = false;
	    Wire.write(0x51);;
	    Wire.endTransmission(1);
	    tick = millis();
	    ping = false;

  }

  if (!ping && millis() - tick > 5) {
    Wire.beginTransmission(I2C[sensorId]);
    Wire.write(0x03);
    Wire.endTransmission(1);
      char v = readI2C(I2C[sensorId]);
       /*if(sensorId==0){
       	Serial2.println(" delay "+String(millis()-times));
     	 times=millis();
      }*/
      SensorData[2 + sensorId] = ((v <= 50 && v != 0 ? v : 0) / 2 & 31)| ((2+sensorId)<<5);
     
      /*if(sensorId==0)
      {Serial.println((int)( SensorData[2 + sensorId]&31)*2);
      times=millis();
      }*/
    ping = true;
    sensorId = 0;//(sensorId + 1) % 2;
  }

  for (int i = 1; i < sizeof(SensorData); i++) {
	
	Serial.println(SensorData[i]);
   // Serial.println(String((SensorData[5]&31)*2)+" id "+String((SensorData[5]>>5)&7));
  }



  unsigned int steeringCH = pulseIn(rcController, HIGH);
  //steering.write(steeringCH);
  //min <90   max >180
  unsigned int speedCH = pulseIn(speedPin, HIGH);
  //esc.write(speedCH);

  //reRead:
  if (Serial.available()) {
    char k = Serial.read();
    byteDecode(k, &speeds, &value);
    Serial2.println("angle is"+String(value));
   // Serial.println(k);
    car->setAngle(value + 120);
    
  } else {
    //Serial2.println("not available####3");

  }

	
  if (speedCH / 10 < 190) {
    car->setRawSpeed(&speedLimit, speedCH);
  } else {

    car->setSpeed(car->NEUTRAL);
    //command='c';
    car->setAngle(90);
  }


	


	   Serial2.println(" delay is"+String(millis()-times));


}






int speedLimit(int speed) {
  if (speed / 10 < 140 && speed / 10 > 130) { //if in neutral
    return 1387;
  } else if (speed / 10 > MAXFORWSPEED) { //max speed forward
    return MAXFORWSPEED * 10;
  } else if (speed / 10 < MAXREVSPEED) { //max spedd in reverse

    return MAXREVSPEED * 10;
  } else {
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


