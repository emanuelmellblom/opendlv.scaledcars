#include <SharpIR.h>

int frontRightIrPin = A3;
int backRightIrPin = A4;
int backIrPin = A5;

//Instansiate the 3 ir sensors using the SharpIR library
SharpIR frontRightIrSensor(GP2YA41SK0F, frontRightIrPin);
SharpIR backRightIrSensor(GP2YA41SK0F, backRightIrPin);
SharpIR backIrSensor(GP2YA41SK0F, backIrPin);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  //Read sensorvalues (detected distance);
  int distanceFrontRightIr = frontRightIrSensor.getDistance(); //Calculate the distance in centimeters and store the value in a variable
  int distanceBackRightIr = backRightIrSensor.getDistance();
  int distanceBackIr = backIrSensor.getDistance();

  //Print the value readings to the Serial monitor
  Serial.println("Front Right IR");
  Serial.println(distanceFrontRightIr);//Print the value to the serial monitor
  delay(1000);
  Serial.println("");
  Serial.println("Back right IR");
  Serial.println(distanceBackRightIr);
  delay(1000);
  Serial.println("");
  Serial.println("Back IR");
  Serial.println(distanceBackIr);
  delay(1000);
  Serial.println("");
  
}
