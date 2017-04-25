#include <Wire.h>
#include <Servo.h>
#include <math.h>
//Ultrasonic sensors
#define ultrasonic_Front_Center 0x72
#define ultrasonic_Front_Side 0x73

//Ir Sensors
#define ir_right_front A3
#define ir_right_rear A4
#define ir_rear A5

//Led lights
#define led_lights 9

//Odometer
#define odometer 6

#define MAXSPEED 150
#define MINSPEED 100
#define rcController 4
#define speedPin 2
//esc
Servo esc;

//Steering servo
Servo steering;

char command='c';
unsigned int value;


void setup() {
  //Inputs and outputs
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
  //esc.writeMicroseconds(50);
  
 
  //Serial
  Serial.begin(9600);

  //I2C (Ultrasonic sensors)
  Wire.begin(ultrasonic_Front_Center);
  Wire.begin(ultrasonic_Front_Side);
    
}

unsigned int powI(int base,int exponent){
	unsigned int sum=1;
	unsigned long tmp=1;
	for(int i=0;i<exponent;i++){
		if((tmp=tmp*base)<=65535)
			sum=tmp;
		else
			return 0;
		
		
	}
return sum; 
}


bool breaking=false;

unsigned int charBufferToInt(char str[],int len){
	int n,sum;
	unsigned long tmp;
	sum=tmp=0;
	n=--len;
	while(n>=0){
	//Serial.println(" bannaa "+String((unsigned long)(powI(10,len-(n)))));
    //Serial.println(" powpow "+String((unsigned long)(str[n]-48)*(powI(10,(len)-n)))); 
		if((tmp=tmp+((unsigned long)(str[n]-48)*(powI(10,(len)-(n--)))))<=65535)
			sum=tmp;
		else
			return sum;    
	}
return sum;
}

bool isNumber(char c){
	return c >= 48 && c <= 57;
}

bool isLetter(char c){
	return c >= 92 && c <= 122;
	
}

void loop() {
	//delay(300);
	//min <90   max >180 
	unsigned int steeringCH=pulseIn(rcController,HIGH); 
	//steering.write(steeringCH);
	//min <90   max >180 
	unsigned int speedCH=pulseIn(speedPin,HIGH);
	//esc.write(speedCH);
  
 	//Serial.println(String(speedCH));
  
  
	if(Serial.available()){
		char inputValue[10]={'\0'};
		command=' ';
		int valueLen=0;
     	value=0;
		char current;
		while (Serial.available()) {
			isNumber(current=Serial.read())?inputValue[valueLen<10?valueLen++:valueLen]=current:command=isLetter(current)?current:isLetter(command)?command:'?';  //gets one byte from serial buffer
		}
		command=isLetter(command)?command:'?';
		value=charBufferToInt(inputValue,valueLen);
		Serial.println("command is "+String(command)+" value is "+String(value));
  }


	/*
		Controller
	*/

	//Serial.println(String(speedCH));
	if(speedCH/10 < 190){
		if(command=='c'){
			Serial.println(String(speedCH));
			steering.write((steeringCH/10)*10);	
			if(speedCH/10 < 140 && speedCH/10 > 130){//if in neutral
				breaking=false;
	        	esc.write(1387);
				Serial.println("stop");
			}else if(speedCH/10 >MAXSPEED){//max speed forward
				breaking=true;
				Serial.println("above MAX");
				esc.write(MAXSPEED*10);
				//speedCH=138*10;
		    }else if(speedCH/10 < MINSPEED){//max spedd in reverse
				breaking=false;
				Serial.println("below MIN");
				esc.write(MINSPEED*10);
		    }else{
		    	Serial.println("go forward");
		    	if(speedCH/10>140){
		    		breaking=true;	
		    	}else{
		    		breaking=false;	
		    	}
	      		esc.write(speedCH);
	     	}
     	}else if(command=='m'){
     		steering.write(90);
     		esc.write(value);
     		if(value/10>140){
		    		breaking=true;	
		    }else{
		    		breaking=false;	
		    }
     	}else if(command=='a'){
     		esc.write(1350);
     		steering.write(value);
     	}else{
     		if(breaking){
  				esc.write(800);
  				delay(100);
  			}
			steering.write(90);
  			breaking=false;
     		esc.write(1350);
     	}
  }else{
  	if(breaking){
  		esc.write(800);
  		delay(100);
  	}
  	esc.write(1350);
  	command='c';
	steering.write(90);
  	breaking=false;
  	esc.write(1387);
  	Serial.println("no controller conneced");
    //esc.write(1387);
  }
  
 

}
