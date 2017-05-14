
#include "RcCar.h"

RcCar::RcCar(int maxForwSpeed,int minForwSpeed,int maxRevSpeed,int minRevSpeed, int neutral,Servo esc,Servo steeringServo):NEUTRAL(neutral), minForwSpeed(minForwSpeed),maxForwSpeed(maxForwSpeed),maxRevSpeed(maxRevSpeed),minRevSpeed(minRevSpeed){
	this->esc=esc;
	this->steering=steeringServo;
	time=millis();
}

RcCar::~RcCar(){
	
}


void RcCar::setAngle(int angle){
	steering.write(angle);

}

/*
 * Not possible to dereferenbce neative values
 * */
int  RcCar::getRatio(float val){
	if(val==0) return this->NEUTRAL;
	if(val> 0&& val<=2) return minForwSpeed+((maxForwSpeed-minForwSpeed)*(val/2.0));
	float x;
	if(val < 0&& val>=-2) return  minRevSpeed+((maxRevSpeed-minRevSpeed)*(-val/2.0));
	
	return this->NEUTRAL;
	//maxRevSpeed+((x=((maxRevSpeed-minRevSpeed)*(val/2.0)))<0?x*-1:x);
	
	
}

bool RcCar::setSpeed(float speed){
	int x=getRatio(speed);
	//Serial.println(x);
	
	bool v=this->Break(&x);	
	this->currentSpeed=x;
	return v;
}



void RcCar::setRawSpeed(int(*fptr)(int speed),int speed){
	speed=fptr(speed);
	esc.write(speed);
	this->currentSpeed=speed;
}



bool RcCar::Break(int* speed){
	if(*speed < this->minForwSpeed && this->currentSpeed > minForwSpeed){
		esc.write(getRatio(-2));
		delay(100);
		esc.write(this->NEUTRAL);	
		delay(100);
		esc.write(*speed);
		delay(100);
		//Serial.println("hellow world "+String(*speed));
		return true;
	}
	//Serial.println(String(this->currentSpeed)+" "+String(this->maxRevSpeed)+" "+String(this->minRevSpeed)+" ");
	if((!(*speed <= this->minRevSpeed && *speed >= this->maxRevSpeed) &&  !(*speed >= this->minRevSpeed && *speed <= this->maxRevSpeed)) && ((this->currentSpeed >= minRevSpeed && this->currentSpeed <= maxRevSpeed) || (this->currentSpeed <= minRevSpeed && this->currentSpeed >= maxRevSpeed))){
		
		
		esc.write(getRatio(2));
		delay(100);
		esc.write(this->NEUTRAL);
		delay(100);
		esc.write(*speed);
		delay(100);
		return true;
	}
	esc.write(*speed);
	return false;
}  


bool RcCar::isValidMoveSpeed(int speed){
	//Serial2.println(speed);
	if((speed<=minRevSpeed && speed>=maxRevSpeed ) || (minForwSpeed<=speed && speed <= maxForwSpeed)){
		return true;
		
	}
	
	return false;


}


unsigned int RcCar::powI(int base,int exponent){
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

unsigned int RcCar::charBufferToInt(char str[],int len){
	int n,sum;
	unsigned long tmp;
	sum=tmp=0;
	n=--len;
	while(n>=0){
		if((tmp=tmp+((unsigned long)(str[n]-48)*(this->powI(10,(len)-(n--)))))<=65535)
			sum=tmp;
		else
			return sum;    
	}
	return sum;
}

bool RcCar::isNumber(char c){
	return c >= 48 && c <= 57;
}

bool RcCar::isLetter(char c){
	return c >= 92 && c <= 122;
	
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

float RcCar::charBufferToFloat(char str[],int len){
	int n,offset,sum;
	unsigned long tmp;
	float sums=0;
	offset=sum=tmp=0;
	n=--len;
	while(n>=0){
		if(str[n]==46 ){
			
			sums=sums+sum;
			//Serial.println("offset "+String(sum));
			tmp=0;
			offset=len-n--+1;
			
		}else{
			if((tmp=tmp+((unsigned long)(str[n]-48)*(powI(10,((len)-((n--)+offset))))))<=65535)
			sum=tmp;
			else
				return sum;    
		}  
	}
	sums=(float)sum+sums/powI(10,offset-1);
return sums;
}







