#include <Wire.h>
#include <Servo.h>
#include <RcCar.h>
//#include <math.h>

#define ultrasonic_Front_Center 112
#define ultrasonic_Front_Side 115

//Ir Sensors
#define ir_right_front A3
#define ir_right_rear A4
#define ir_rear A5

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

//Steering servo
Servo steering;

//char command;
float value;
float speeds;

RcCar* car ;






int speedLimit(int speed){
	if(speed/10 < 140 && speed/10 > 130){//if in neutral
	       return 1387;
	}else if(speed/10 >MAXFORWSPEED){//max speed forward
		return MAXFORWSPEED*10;
	}else if(speed/10 < MAXREVSPEED){//max spedd in reverse
				
		return MAXREVSPEED*10;
	}else{
	      		 return speed;
	     	}
	     
}


void byteDecode(int byteIn,float *speed,float * angle){
	float tempAng=*angle;
	*angle=((byteIn&31)-15)*4;
	*speed=(((byteIn&224)>>5)-3)/2;

	if(*angle==48){
		//command='a';
		*angle=tempAng;
		
	}
	
	
	
}






void setup() {
//	command='c';
	speeds=0;
	value=90;
	//Inputs and outputs
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(ir_right_front, INPUT);
	pinMode(ir_right_rear, INPUT);
	pinMode(ir_rear, INPUT);
	pinMode(led_lights, OUTPUT);
	pinMode(odometer, INPUT);
	pinMode(rcController,INPUT);
	pinMode(speedPin,INPUT);
   
  //Servos
  esc.attach(10);
  steering.attach(3);
  esc.write(1380);
  
  car =new RcCar(MAXFORWSPEED*10,1400,MAXREVSPEED*10,1100,1380,esc,steering);
  //esc.writeMicroseconds(50);
  
 Wire.begin();
  //Serial
  Serial.begin(9600);

  

	
}





int readI2C(int address){
	//Serial.println(String(Wire.requestFrom(address,1)));
	Wire.requestFrom(address,1);
	while(Wire.available()){
	 	
	 	char x=Wire.read();
	 
	 	//if(x>0)Serial.println("pinged "+String((int)x));
	 	
	 	
	}
	 return -1;

} 


unsigned long tick=0;
boolean ping=true;
char sensorId=0;
char I2C[2]= {ultrasonic_Front_Center,ultrasonic_Front_Side};
void loop() {


	// Serial.println("cae");
	  unsigned long times=millis();
	  
	  	/*if(ping){	
		  	Wire.beginTransmission(I2C[sensorId]);
				Wire.write(0x02);
				Wire.write(10);
			Wire.endTransmission(1);
			//delay(20);
		  	Wire.beginTransmission(I2C[sensorId]);
				Wire.write(0x00);
				Wire.write(0x51);;
			Wire.endTransmission(1);
			tick=millis();	
			ping=false;
			
	  	}
	  	
	  	if(!ping&&millis()-tick>5){
			Wire.beginTransmission(I2C[sensorId]);
				Wire.write(0x03);
			Wire.endTransmission(1);
			for(int i=0;i<1;i++){
	  			readI2C(I2C[sensorId]);
		 	}
			ping=true;
			sensorId=(sensorId+1)%2;
	  	}*/

		
		/*for(int i=0;i<128;i++){
			 Wire.requestFrom(i,1);
			 if(Wire.available())Serial.println(i);
		}*/

		unsigned int steeringCH=pulseIn(rcController,HIGH); 
		//steering.write(steeringCH);
		//min <90   max >180 
		unsigned int speedCH=pulseIn(speedPin,HIGH);
		
		//esc.write(speedCH);
	  
	 	//Serial.println(String(speedCH));
	 // Serial2.println("hellow world");
		
	//reRead:
	if(Serial.available()){
		char k=Serial.read();
		byteDecode(k,&speeds,&value);
		car->setAngle(value+120);		
		//Serial2.println("uberangle "+String((int)k));
		//Serial2.println("angle "+String(value));
		//Serial2.println("car angle "+String(value+120));
		//Serial.println("angle "+String(value));
		//Serial.flush();
		//Serial2.flush();
	}else{
		//Serial2.println("not available####3");
		
	}


	if(speedCH/10 < 190){
		car->setRawSpeed(&speedLimit,speedCH);
	}else{

		car->setSpeed(car->NEUTRAL);
		//command='c';
		car->setAngle(90);
	}

	Serial.println(" time" +String(millis()-times));
	
	/*
  
	if(Serial.available()){
		char inputValue[10]={'\0'};
		command=' ';
		int valueLen=0;
     	//value=0;//removed this
     	int tick=0;//added this ticking hopefully this will prevent the system getting stuck in the buffer
     	int state=1;
		char current;
		bool notCompletedMessage=false;
		Redo:
		while (Serial.available()&&(notCompletedMessage=((current=Serial.read())!='&'))&& tick++<6) {
			isFloat(current)?inputValue[valueLen<10?valueLen++:valueLen]=current:current=='-'?state=-1:command=isLetter(current)?current:isLetter(command)?command:'?';  //gets one byte from serial buffer
			Serial2.println("input is "+String(current)+" or "+String((int)current)+" tick is "+String(tick));
		}

		//if(notCompletedMessage){
			//goto Redo;	
		//}
		///Serial.flush();//added this flush memory
		command=isLetter(command)?command:'?';
		//Serial.println(String(inputValue));
		value=car->charBufferToFloat(inputValue,valueLen)*state;
		Serial2.println("command is "+String(command)+" value is "+String(value));
  }


		



	if(speedCH/10 < 190){
		if(command=='c'){
			//Serial.println(String(speedCH));
			car->setAngle((steeringCH/10)*10);
	     	car->setRawSpeed(&speedLimit,speedCH);
	     	delay(110);
     	}else if(command=='m'){
     		steering.write(90);
     		car->setSpeed(value);
     	
     	}else if(command=='a'){
     	
     		//esc.write(1350);
     		//steering.write(value);
     		car->setRawSpeed(&speedLimit,speedCH);
     		//command='c';
     		//maybe we should add a turn limit
     		car->setAngle(value);
     	}else{
     		car->setSpeed(car->NEUTRAL);
     		car->setAngle(100);
     	
		}
	}else{
		
		//esc.write(1350);
		car->setSpeed(car->NEUTRAL);
		command='c';
		car->setAngle(110);
	
		Serial.println("no controller conneced");
	}*/
  
 

}











bool isFloat(char c){
	return (c >= 48 && c <= 57) || c==46;
}

bool isNumber(char c){
	return c >= 48 && c <= 57;
}

bool isLetter(char c){
	return c >= 92 && c <= 122;
	
}


