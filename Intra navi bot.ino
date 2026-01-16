#include<SoftwareSerial.h>
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define pwm_a 10
#define pwm_b 5
#define xaxis A0
#define yaxis A1


SoftwareSerial bluetooth (11,12);


const int motor_speed = 500;


void setup(){
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(pwm_a,OUTPUT);
  pinMode(pwm_b,OUTPUT);
  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  Serial.begin(9600);
  bluetooth.begin(9600);
}
//MOTOR SPEED CONTROL
void setmotorspeed(int speed) {
  analogWrite(pwm_a, speed);
  analogWrite(pwm_b, speed);
  TCCR0B = (TCCR0B & 0b11111101 )| 0x04; // INTERNAL TIMER CONTROL
  TCCR1B = (TCCR1B & 0b11111101) | 0x04;
}


void fwd(){
  digitalWrite(in1,1);
  digitalWrite(in2,0);
  digitalWrite(in3,1);
  digitalWrite(in4,0);
  setmotorspeed(motor_speed);
}


void rev(){
  digitalWrite(in1,0);
  digitalWrite(in2,1);
  digitalWrite(in3,0);
  digitalWrite(in4,1);
  setmotorspeed(motor_speed);
}


void right(){
  digitalWrite(in1,0);
  digitalWrite(in2,0);
  digitalWrite(in3,1);
  digitalWrite(in4,0);
  setmotorspeed(motor_speed);
}


void left(){
  digitalWrite(in1,1);
  digitalWrite(in2,0);
  digitalWrite(in3,0);
  digitalWrite(in4,0);
  setmotorspeed(motor_speed);
}


void stop(){
  digitalWrite(in1,0);
  digitalWrite(in2,0);
  digitalWrite(in3,0);
  digitalWrite(in4,0);
  setmotorspeed(0);
}


char command;
void loop(){ 
 
//JOYSTICK CONTROL


  int x=analogRead(xaxis);
  Serial.print("x =");
  Serial.println(x);
  int y=analogRead(yaxis);
  Serial.print("| y =");
  Serial.println(y);
 


  if(x<=100) fwd();
  if(x>=800) rev();
  if(y<=400)  right();
  if(y>=600) left();


//SERIAL MONITOR


  if(Serial.available()){
      command=Serial.read();
  if(command=='f')
  {
    fwd();
    Serial.println('f');}
  if(command=='b') {
    rev();
    Serial.println('b');}
  if(command=='r') {right();
  Serial.println('r');}
   if(command=='l') {left();
  Serial.println('l');}
  if(command=='s') stop();
  }
//BLUETOOTH CONTROL


  if(bluetooth.available()){
    command=bluetooth.read();
  if(command=='f') fwd();
  if(command=='b') rev();
  if(command=='r') right();
  if(command=='l') left();
  if(command=='s') stop(); }}


TO MEASURE THE DISTANCE THROUGH THE ENCODER


//PIN DEFINITION
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define pwm_a 10
#define pwm_b 5


volatile long encoderCount = 0;
const int encoderPinA = 4;  
const int encoderPinB = 3;
const int frequency = 1000;//TO CONTROL THE PWM
const int dutyCycle = 127;
long previousCount = 0;
float distancePerThreshold = 0;


const int motor_speed = 500;


void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = (F_CPU / frequency) - 1;
  OCR1A = (ICR1 * dutyCycle) / 255;
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(pwm_a,OUTPUT);
  pinMode(pwm_b,OUTPUT);
  Serial.begin(9600);}


// ENCODER COUNT
void updateEncoder() {
  bool A = digitalRead(encoderPinA);
  bool B = digitalRead(encoderPinB);


  if (A == B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}


void setmotorspeed(int speed) {
  analogWrite(pwm_a, speed);
  analogWrite(pwm_b, speed);
  TCCR0B = (TCCR0B & 0b11111101 )| 0x04;
  TCCR1B = (TCCR1B & 0b11111101) | 0x04;
}


void fwd(){
  digitalWrite(in1,1);
  digitalWrite(in2,0);
  digitalWrite(in3,1);
  digitalWrite(in4,0);
  setmotorspeed(motor_speed);
}


void rev(){
  digitalWrite(in1,0);
  digitalWrite(in2,1);
  digitalWrite(in3,0);
  digitalWrite(in4,1);
  setmotorspeed(motor_speed);
}


void right(){
  digitalWrite(in1,1);
  digitalWrite(in2,0);
  digitalWrite(in3,0);
  digitalWrite(in4,1);
  setmotorspeed(motor_speed);
}


void left(){
  digitalWrite(in1,0);
  digitalWrite(in2,1);
  digitalWrite(in3,1);
  digitalWrite(in4,0);
  setmotorspeed(motor_speed);
}


void stop(){
  digitalWrite(in1,0);
  digitalWrite(in2,0);
  digitalWrite(in3,0);
  digitalWrite(in4,0);
  setmotorspeed(0);
}
char command;
void loop() {
  long currentCount;
  currentCount = encoderCount;
  long diff = abs(currentCount - previousCount);
 if (diff >= 0.0906) {
    Serial.print("Traveled ");
    Serial.print(distancePerThreshold);
    Serial.println(" cm");
  while(distancePerThreshold==100){
  stop();
  fwd();
  }
  previousCount = currentCount;
  distancePerThreshold=distancePerThreshold+0.906;


   


  }


  //PRINTING THE VALUES OF THE ENCODER
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
 
  Serial.print("pwm_a= ");
  Serial.print(analogRead(pwm_a));
  Serial.print("pwm_b ");
  Serial.print(analogRead(pwm_b));
  Serial.print("Distance Per Threshold=");
  Serial.println(distancePerThreshold);


  if(Serial.available()){
      command=Serial.read();
  if(command=='f')
  {
    fwd();
    Serial.println('f');}
  if(command=='b') {
    rev();
    Serial.println('b');}
  if(command=='r') {right();
  Serial.println('r');}
   if(command=='l') {left();
  Serial.println('l');}
  if(command=='s') stop();
}}


