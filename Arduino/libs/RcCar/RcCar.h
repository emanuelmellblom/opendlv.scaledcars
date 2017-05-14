#ifndef _RcCae_H_
#define _RcCar_H_


#include <Servo.h>
#include <Arduino.h>
class RcCar {
		public:
			/*
			 * Flags
			 * */
			const int NEUTRAL;
			/*
			 * car control
			 * */
			RcCar(int maxForwSpeed,int minForwSpeed,int maxRevSpeed,int minRevSpeed, int neutral,Servo escPin,Servo steeingServoPin);
			virtual ~RcCar();
			bool setSpeed(float speed);
			void setRawSpeed(int(*fptr)(int speed),int speed);
			void setAngle(int);	 
			bool isValidMoveSpeed(int);
			/*
			 * Other functions
			 * */
			
			unsigned int powI(int base,int exponent);/*Extension of the math lib*/
			unsigned int charBufferToInt(char str[],int len);/*Extension of the String lib*/
			int getRatio(float );
			bool isNumber(char c);
			bool isLetter(char c);
			float charBufferToFloat(char str[],int len);
		private:
			const int maxForwSpeed;
			const int minForwSpeed;
			const int maxRevSpeed;
			const int minRevSpeed;
			unsigned long time;
			int currentSpeed;
			Servo esc;
			Servo steering;
			
			bool Break(int *speed);
			
			
			
	
};

#endif  // _RcCar_H_



