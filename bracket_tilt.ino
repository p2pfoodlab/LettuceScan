#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *TiltMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *PanMotor = AFMS.getStepper(200, 1);

char command;
char dir;
int value;
int p=0;
int t=0;

int mode=DOUBLE; //SINGLE 1, DOUBLE 2, INTERLEAVE 3, MICROSTEP 4

void setSpeed(int s){
 TiltMotor->setSpeed(s);    
 PanMotor->setSpeed(s);  
}


void setup() {
 Serial.begin(9600);             
 AFMS.begin();
 
 TiltMotor->setSpeed(50);    
 PanMotor->setSpeed(50);  
 
 TiltMotor->step(0, FORWARD, mode);
 PanMotor->step(0, FORWARD, mode); 
 
 }
 
void loop(){
   //TiltMotor->step(10, BACKWARD, mode);
   //TiltMotor->step(10, FORWARD, mode);
   while (Serial.available() > 0) {
       command = Serial.read();
       value = Serial.parseInt();
       Serial.read(); //TODO: parse terminaison signal
       Serial.print(command);
       Serial.println(value);

       switch(command){
          case 'p': 
             PanMotor->step(abs(value), (2-(value>0)), mode); 
             p+=value;
             break;    
          case 't': 
             TiltMotor->step(abs(value), (2-(value>0)), mode);
             t+=value;
             break;
          case 'm': mode=value;
          case 's': setSpeed(value);
       }   
   }
}
