#define MOTORPIN1 11
#define MOTORPIN2 12

#define waitTimeSensorMS 5000
#define waitTimeMotorMS 5000
int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1);
  
  //toggle Heartbeat LED
  pinMode(LED_BUILTIN, OUTPUT);
  //digital out for Relay 7,8
  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on 
  delay(500);                       // wait for half a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off 
  delay(500);   

  digitalWrite(MOTORPIN1, HIGH);
  digitalWrite(MOTORPIN2, HIGH);

  // wait for serial cmd
  //while (!Serial.available());

  // get serial cmd
  x = Serial.readString().toInt();

  // valid cmd
  if(x == 1){
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
    
    delay(waitTimeSensorMS);

    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);

    delay(waitTimeMotorMS);

    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, HIGH);
    
    Serial.print(x+1);

    x = 0;
  }
  

}
