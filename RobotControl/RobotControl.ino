
#include <Servo.h>

Servo myservo;  // create servo object to control rack
Servo myservo2; //Servo for the sensor package
// twelve servo objects can be created on most boards

int pos = 0;
/*
* Brushed_H_Bridge sketch
* commands from serial port control motor speed and direction
* digits '0' through '9' are valid where '0' is off, '9' is max speed
* + or - set the direction
*/
const int enPinA = 5; // H-Bridge enable pin
const int in1Pin = 7; // H-Bridge input pins
const int in2Pin = 4;
const int enPinB = 6; // H-Bridge enable pin
const int in3Pin = 3; // H-Bridge input pins
const int in4Pin = 2;
const int servoPin = 9; //Signal for myservo (control rack)
const int servo2Pin = 10; //Signal for myservo2 (control Sensor Package)
int incomingByte = 0;   // for incoming serial data

void setup()
{
                myservo.attach(9);  // attaches the servos on pins 9 and 10 to they're functions
                myservo2.attach(10);
                int pos = 93;    // variable to store the servo position ---90 is stop for continuous rotation.
              
              
                
                Serial.begin(9600);
                pinMode(in1Pin, OUTPUT);
                pinMode(in2Pin, OUTPUT);
                pinMode(in3Pin, OUTPUT);
                pinMode(in4Pin, OUTPUT);
                pinMode(servoPin, OUTPUT);
                pinMode(servo2Pin, OUTPUT);
                Serial.println("Speed (0-9) or + - to set direction"); 
                Serial.println("'l' to move rack out, 'r' in, and '<' stop");
                Serial.println("'z' to rotate SP CW, 'x' to rotate CCW, '>' to stop it");
}
void loop()
{
      if (Serial.available() > 0) {
         // read the incoming byte:               
                incomingByte = Serial.read();
                Serial.println(incomingByte);// say what you got:
                Serial.print("received: ");
                Serial.print (incomingByte);
                
                //CONTROLS FOR THE SENSOR PACKAGE ROTATION
                if(incomingByte == 108){ 
                     Serial.println(" sent l Rotaing CW "); 
                     myservo.write(91); 
                }else if(incomingByte == 114){
                     Serial.println(" sent r Rotaing CCW "); 
                      myservo.write(95); 
                }else if(incomingByte == ','){
                      Serial.println(" sent ',' Stopped "); 
                      myservo.write(93); 
                }

                //CONTROLS FOR THE RACK EXTENSION  
                else if(incomingByte == 'z'){
                     Serial.println(" sent z Rotaing SP CW "); 
                     myservo2.write(99);
                }
                  else if(incomingByte == 'x'){
                     Serial.println(" sent x Rotaing SP CCW "); 
                     myservo2.write(87);  
                }else if(incomingByte == '.'){
                      Serial.println(" sent '.' Stopped "); 
                      myservo2.write(93); 
                }  

                //CONTROLS FOR THE ROBOT MOTION
                if(incomingByte >= '0' && incomingByte <= '9') // is this a speed?
                {
                      int speed = map(incomingByte, '0', '9', 0, 255);
                      analogWrite(enPinA, speed);
                      analogWrite(enPinB, speed);
                      Serial.println(speed);
                }
                else if (incomingByte == '=') //Configure to move forward
                {
                      Serial.println("CW");
                      digitalWrite(in1Pin,LOW);
                      digitalWrite(in2Pin,HIGH);
                      digitalWrite(in3Pin,LOW);
                      digitalWrite(in4Pin,HIGH);
                }
                else if (incomingByte == '-') //Configure to move backwards
                {
                      Serial.println("CCW");
                      digitalWrite(in1Pin,HIGH);
                      digitalWrite(in2Pin,LOW);
                      digitalWrite(in3Pin,HIGH);
                      digitalWrite(in4Pin,LOW);
                }
                else
                {
                    Serial.print("Unexpected character ");
                    Serial.println(incomingByte);
                }
      }
}
