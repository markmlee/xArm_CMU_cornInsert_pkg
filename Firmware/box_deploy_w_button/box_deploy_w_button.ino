#include <Servo.h>

Servo myservo;  
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

const int buttonPin1 = 2;
const int buttonPin2 = 3;
const int buttonPin3 = 4;
const int buttonPin4 = 5;
const int buttonPin5 = 6;

int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;
int buttonState5 = 0;

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 2000;    // the debounce time; increase if the output flickers

int boxState1 = -1;
int boxState2 = -1;
int boxState3 = -1;
int boxState4 = -1;
int boxState5 = -1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(8);  
  myservo2.attach(9);
  myservo3.attach(10);
  myservo4.attach(11);
  myservo5.attach(12);

  myservo.write(55);
  myservo2.write(55);
  myservo3.write(55);
  myservo4.write(55);
  myservo5.write(55);  

  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(buttonPin4, INPUT);
  pinMode(buttonPin5, INPUT);
}

void loop() {

  if ( (millis() - lastDebounceTime) > debounceDelay) {

    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    buttonState4 = digitalRead(buttonPin4);
    buttonState5 = digitalRead(buttonPin5);
    
    if ( (buttonState1 == 1) && (boxState1 < 0) ) {
      Serial.println("open door 1");
      myservo.write(0); //open the box
      boxState1 = -boxState1; //change the state of box
      lastDebounceTime = millis(); //set the current time      
    }
    else if ( (buttonState1 == 1) && (boxState1 > 0) ) {
      Serial.println("closed door 1");
      myservo.write(55); //close the box
      boxState1 = -boxState1; //change the state of the box
      lastDebounceTime = millis(); //set the current time
    }

    if ( (buttonState2 == 1) && (boxState2 < 0) ) {
      Serial.println("open door 2");
      myservo2.write(0); //open the box
      boxState2 = -boxState2; //change the state of box
      lastDebounceTime = millis(); //set the current time
      }
    else if ( (buttonState2 == 1) && (boxState2 > 0) ) {
      Serial.println("closed door 2");
      myservo2.write(55); //close the box
      boxState2 = -boxState2; //change the state of the box
      lastDebounceTime = millis(); //set the current time
    }

    if ( (buttonState3 == 1) && (boxState3 < 0) ) {
      Serial.println("open door 3");
      myservo3.write(0); //open the box
      boxState3 = -boxState3; //change the state of box
      lastDebounceTime = millis(); //set the current time
      }
    else if ( (buttonState3 == 1) && (boxState3 > 0) ) {
      Serial.println("closed door 3");
      myservo3.write(55); //close the box
      boxState3 = -boxState3; //change the state of the box
      lastDebounceTime = millis(); //set the current time
    }

    if ( (buttonState4 == 1) && (boxState4 < 0) ) {

      Serial.println("open door 4");
      myservo4.write(0); //open the box
      boxState4 = -boxState4; //change the state of box
      lastDebounceTime = millis(); //set the current time
      
      }
    else if ( (buttonState4 == 1) && (boxState4 > 0) ) {

      Serial.println("closed door 4");
      myservo4.write(55); //close the box
      boxState4 = -boxState4; //change the state of the box
      lastDebounceTime = millis(); //set the current time
    }

    if ( (buttonState5 == 1) && (boxState5 < 0) ) {

      Serial.println("open door 5");
      myservo5.write(0); //open the box
      boxState5 = -boxState5; //change the state of box
      lastDebounceTime = millis(); //set the current time
      }
    else if ( (buttonState5 == 1) && (boxState5 > 0) ) {
      
      Serial.println("closed door 5");
      myservo5.write(55); //close the box
      boxState5 = -boxState5; //change the state of the box
      lastDebounceTime = millis(); //set the current time
    }

  }//close if(time buffer)

}//close void loop
