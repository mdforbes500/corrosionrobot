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
void setup()
{
Serial.begin(9600);
pinMode(in1Pin, OUTPUT);
pinMode(in2Pin, OUTPUT);
pinMode(in3Pin, OUTPUT);
pinMode(in4Pin, OUTPUT);
Serial.println("Speed (0-9) or + - to set direction"); 
Serial.println("< control only left side, > control only right side");
}
void loop()
{
if ( Serial.available()) {
char ch = Serial.read();

if(ch >= '0' && ch <= '9') // is ch a number?
{
int speed = map(ch, '0', '9', 0, 255);
analogWrite(enPinA, speed);
analogWrite(enPinB, speed);
Serial.println(speed);
}
else if (ch == '+')
{
Serial.println("CW");
digitalWrite(in1Pin,LOW);
digitalWrite(in2Pin,HIGH);
digitalWrite(in3Pin,LOW);
digitalWrite(in4Pin,HIGH);
}
else if (ch == '-')
{
Serial.println("CCW");
digitalWrite(in1Pin,HIGH);
digitalWrite(in2Pin,LOW);
digitalWrite(in3Pin,HIGH);
digitalWrite(in4Pin,LOW);
}
else if (ch == '<')
{
Serial.println("<");
digitalWrite(in1Pin,HIGH);
digitalWrite(in2Pin,LOW);
analogWrite(enPinB, 0);
}
else if (ch == '>')
{
Serial.println(">");
digitalWrite(in3Pin,HIGH);
digitalWrite(in4Pin,LOW);
analogWrite(enPinA, 0);
}

else
{
Serial.print("Unexpected character ");
Serial.println(ch);
}
}
}
