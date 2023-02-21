#define IN1	3
#define IN2	9
#define IN3	10
#define IN4	11
#include<NewPing.h>// include newping library

NewPing sonar_1 (15, 15, 400);// create ultrasonic object with the following parameters (Trig=3, Echo=2, Max distance=400cm)
float distance_right;// initialize a variable of type float

NewPing sonar_2 (16, 16, 400);// create ultrasonic object with the following parameters (Trig=3, Echo=2, Max distance=400cm)
float distance_front;// initialize a variable of type float

NewPing sonar_3 (17, 17, 400);// create ultrasonic object with the following parameters (Trig=3, Echo=2, Max distance=400cm)
float distance_left;// initialize a variable of type float

int command;            //Int to store app command state.

int state = 0;

int irpin = A5; 
int irpin_right = 4;
int irpin_left = 7;

int a=0;


int led = 5;
int led_1 = 6;

int measurePin = A0;
int ledPower = 8;  

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;



#include <SoftwareSerial.h>
SoftwareSerial SIM(12, 13); 

String textMessage;

String lampState = "HIGH";

unsigned long oldtime = 0;

void setup(){  
  
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  
pinMode(irpin, INPUT);
pinMode(irpin_right, INPUT);
pinMode(irpin_left, INPUT);

pinMode(led, OUTPUT);
digitalWrite(led, LOW);
  

pinMode(led_1, OUTPUT);
digitalWrite(led_1, LOW);


pinMode(ledPower,OUTPUT);


// Initializing serial
  
  SIM.begin(115200);

  
  delay(5000);
  Serial.println("SIM start...");
 
  // AT command 
  SIM.println("AT");
  delay(100);
  SIM.println("AT+CMGF=1"); 
  delay(100);
  //receive data live
  SIM.println("AT+CNMI=2,2,0,0,0");
  delay(100);

} 

void goAhead(){ 
  
  analogWrite(IN1, 0);
	analogWrite(IN2, 255);
  analogWrite(IN3, 0);
	analogWrite(IN4, 255);

  }

void goBack(){ 

  analogWrite(IN1, 255);
	analogWrite(IN2, 0);
  analogWrite(IN3, 255);
	analogWrite(IN4, 0);

  }

void goRight(){ 

  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  analogWrite(IN3, LOW);
  analogWrite(IN4, 0);

}

void goLeft(){
      
  analogWrite(IN1, 0);
  analogWrite(IN2, 255);
  analogWrite(IN3, 255);
  analogWrite(IN4, 0);
        
}

void stopRobot(){  

  analogWrite(IN1, 0);
	analogWrite(IN2, 0);
	analogWrite(IN3, 0);
	analogWrite(IN4, 0);
  
}

void Auto(){
if ((unsigned long) (millis() - oldtime) > 3000) // after 3s
{ 
  analogWrite(led, 255);
  //delay(100);
  analogWrite(led_1, 255);
  //delay(100);
      oldtime = millis();    //mark time for now
  }
  

distance_right = sonar_1.ping_cm();
Serial.print("Distance_right = ");
Serial.print(distance_right);
Serial.println(" cm");
//delay(10);

distance_front = sonar_2.ping_cm();
Serial.print("Distance_front = ");
Serial.print(distance_front);
Serial.println(" cm");
//delay(10);

distance_left = sonar_3.ping_cm();
Serial.print("Distance_left = ");
Serial.print(distance_left);
Serial.println(" cm");
//delay(10);

  int s = analogRead(irpin);
  int S_right = digitalRead(irpin_right);
  int S_left = digitalRead(irpin_left);

  if((s==HIGH))
  { 
    stopRobot();
    delay(1000);
    goBack();
    delay(1500);
    goLeft();
    delay(500);
    
  }

  if((S_left == LOW)|| (S_right == LOW))
  { 
    stopRobot();
    delay(1000);
    goBack();
    delay(1500);
    goRight();
    delay(500);
    
  }

  if((s==LOW)&&(distance_left <= 20 && distance_front <= 20 && distance_right <= 20))
    {
      goLeft();
    }
  if((s==LOW)&&(distance_left <= 20 && distance_front > 20 && distance_right <= 20))
    {
    goAhead();
    }
  if((s==LOW)&&(distance_left > 20 && distance_front > 20 && distance_right > 20))
  {
    goAhead();
  }
  if((s==LOW)&&(distance_left <= 20 && distance_front <= 20 && distance_right > 20) || (s==LOW)&&(distance_left <= 20 && distance_front > 20 && distance_right > 20))
  {//goRight
    goRight();
  }
  if((s==LOW)&&(distance_left > 20 && distance_front <= 20 && distance_right <= 20) || (s==LOW)&&(distance_left > 20 && distance_front > 20 && distance_right <= 20) || (s==LOW)&&(distance_left > 20 && distance_front <= 20 && distance_right > 20))
  {//goLeft
    goLeft();

  }  
}


void highest()
{
  analogWrite(led, 255);
  //delay(100);
  analogWrite(led_1, 255);
  //delay(100);
  }
void medium()
{
  analogWrite(led, 230);
  analogWrite(led_1, 230);
  }
void lowest()
{
  analogWrite(led, 200);
  analogWrite(led_1, 200);
  }
void stop_mos()
{
  analogWrite(led,0);
  analogWrite(led_1,0);
  
  }
 
void turn_around_left()
{
  goLeft();
}

void turn_around_right()
{
  goRight();
}

void dust()
{
  digitalWrite(ledPower,LOW); 
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);


  calcVoltage = voMeasured * (5.0 / 1024.0);

  // linear eqaution 
  dustDensity = 170 * calcVoltage - 0.1;


  Serial.println("Raw Signal Value (0-1023):");
  Serial.println(voMeasured);

  Serial.println("Voltage:");
  Serial.println(calcVoltage);

  Serial.println("Dust Density:");
  Serial.println(dustDensity); 
  if(dustDensity > 600)
  {
    Serial.println("HIGH");
   
    String dust = "Dust density =" + String(dustDensity) + "ug/m3";    
    sendSMS_warning(dust);
    }
  else{
    Serial.println("LOW");
    }
  
  delay(1000);
}

void dust_sensor()
{
  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); 

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); 
  delayMicroseconds(sleepTime);


  calcVoltage = voMeasured * (5.0 / 1024.0);


  dustDensity = 170 * calcVoltage - 0.1;


  Serial.println("Raw Signal Value (0-1023):");
  Serial.println(voMeasured);

  Serial.println("Voltage:");
  Serial.println(calcVoltage);

  Serial.println("Dust Density:");
  Serial.println(dustDensity); 
  
  String dust = "Dust density =" + String(dustDensity) + "ug/m3";    
  sendSMS_warning(dust);
}

void loop(){

  if (Serial.available() > 0){ 
  command = Serial.read();
  
      if (command == 'F') state = 2;
      else if (command == 'B') state = 3;
      else if (command == 'L') state = 4;
      else if (command == 'R') state = 5;
    
      else if (command == 'S') state = 6;

      else if (command == 'A') state = 1;
      
      else if (command == 'P') state = 0;
      
      else if (command == 'O') state = 7;
      else if (command == 'M') state = 8;
      else if (command == 'C') state = 9;
      else if (command == 'I') state = 10;
      else if (command == 'U') state = 11;
      else if (command == 'D') state = 12;
      else if (command == 'G') state = 13;

      else if (command == 'Z') state = 14;

      }

if(state == 1)
{
  Auto();
  }
  else if(state == 2)
  {
    goAhead();
    }
    else if(state == 3)
  {
    goBack();
    }
    else if(state == 4)
  {
    goLeft();
    }
    else if(state == 5)
  {
    goRight();
    }
    else if(state == 6)
  {
    stopRobot();
    }
    else if(state == 7)
  {
    highest();
    }
    else if(state == 8)
  {
    medium();
    }
    else if(state == 9)
  {
    lowest();
    }
    else if(state == 10)
  {
    turn_around_left();
    }
    else if(state == 11)
  {
    turn_around_right();
    }
    else if(state == 12)
  {
    dust_sensor();
    }
    else if(state == 13)
  {
    dust();
    }
  else if(state == 14)
  {
    //zigzag();
    //combine();
    
  }
  
  else
  {
    stop_mos();
    }
 

if(SIM.available()>0){
    textMessage = SIM.readString();
    Serial.print(textMessage);    
    delay(10);
  } 
  if(textMessage.indexOf("6541")>=0){
    
    goAhead();
    lampState = "goAhead";
    textMessage = "";   
  }
  if(textMessage.indexOf("6642")>=0){
    
    goBack();
    lampState = "goBack"; 
    textMessage = ""; 
  }
  if(textMessage.indexOf("7643")>=0){
    
    goLeft();
    lampState = "goLeft"; 
    textMessage = ""; 
  }
  if(textMessage.indexOf("8252")>=0){
    
    goRight();
    lampState = "goRight"; 
    textMessage = ""; 
  }
if(textMessage.indexOf("8353")>=0){
  
    lampState = "Stop";
    stopRobot();   
    textMessage = ""; 
  }
if(textMessage.indexOf("2009")>=0){
  
    dust_sensor();   
  }
  if(textMessage.indexOf("8454")>=0){
    
    Auto();
    lampState = "Auto"; 
    textMessage = ""; 
  }
  if(textMessage.indexOf("6743")>=0){
    String message = "State is " + lampState;
    sendSMS(message);
    textMessage = "";
  }


}

// Function that sends SMS
void sendSMS(String message){
  //set SMS mode
  SIM.println("AT+CMGF=1"); 
  delay(100);

  
  SIM.println("AT+CMGS=\"+84868475996\""); 
  delay(100);
 
  SIM.println(message); 
  delay(100);

 
  SIM.write(26); 
  delay(100);
  SIM.println();
  
  delay(5000);  
}

void sendSMS_warning(String dust){
  
  SIM.println("AT+CMGF=1"); 
  delay(100);


  SIM.println("AT+CMGS=\"+84868475996\""); 
  delay(100);
  // Send the SMS
  SIM.println(dust); 
  delay(100);


  SIM.write(26); 
  delay(100);
  SIM.println();

  delay(5000);  
}
