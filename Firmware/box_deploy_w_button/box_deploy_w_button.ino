  #include <Servo.h>

// Define a structure to encapsulate the details of a "box"
struct logger {
  Servo servo;        // the servo associated with the box
  int boxState = -1;  // current state of the box (-1=closed, 1=open)
};

const int NUMBER_OF_BOXES = 5;  // Define the total number of boxes
const int STARTUP_ANGLE = 95;
const int RELEASE_ANGLE = 120;

logger boxes[NUMBER_OF_BOXES];  // Create an array of boxes

// Array to hold button pin numbers
int buttonPins[NUMBER_OF_BOXES] = { 2, 3, 5, 4, 6 };

// Array to hold servo pin numbers
int servoPins[NUMBER_OF_BOXES] = { 11, 12, 9, 10, 8 };
// Variables to control debouncing

long debounceDelay = 50;  // the debounce time; increase if the output flickers

struct button {
  long lastDebounceTime = 0;  // the last time the output pin was toggled
  int pin;                    // the pin for the button that controls the box
  int lastState = 0;          // last state of the button (0=unpressed, 1=pressed)
  int state = 0;
  int reading;
};

button buttons[NUMBER_OF_BOXES];

// A buffer to hold incoming commands from the Serial input
char inputBuffer[200];
int bufferIndex = 0;

void setup() {
  // Loop over each box to set it up
  for (int i = 0; i < NUMBER_OF_BOXES; i++) {
    buttons[i].pin = buttonPins[i];       // assign the button pin from the array
    boxes[i].servo.attach(servoPins[i]);  // attach the servo to the pin from the array
    boxes[i].servo.write(STARTUP_ANGLE);             // move the servo to its initial position
    pinMode(buttons[i].pin, INPUT);   // set the button's pin mode to INPUT
  }
  Serial.begin(9600);  // begin serial communication
}


void loop() {
  // Check if any data is available on the Serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    // If newline character is received, a full command is ready for parsing
    if (inChar == '\n') {
      inputBuffer[bufferIndex] = '\0';  // Null-terminate the C string

      char command = strtok(inputBuffer, " ")[0];  // Take the first character as the command

      switch (command) {
        case 'o':  // Open command
        case 'c':
          {  // Close command
            // For these commands, parse additional arguments
            int boxNumber = atoi(strtok(NULL, " "));  // Parse the box number starting from the third character in the input buffer
            if (boxNumber >= 1 && boxNumber <= NUMBER_OF_BOXES) {
              int boxIndex = boxNumber - 1;
              if (command == 'o') {
                boxes[boxIndex].servo.write(RELEASE_ANGLE);
                boxes[boxIndex].boxState = 1;
                Serial.println("{\"code\": 200, \"explain\": \"Success: Opened box " + String(boxNumber) + "\"}");
              } else {
                boxes[boxIndex].servo.write(STARTUP_ANGLE);
                boxes[boxIndex].boxState = -1;
                Serial.println("{\"code\": 200, \"explain\": \"Success: Closed box " + String(boxNumber) + "\"}");
              }
            } else {
              Serial.println("{\"code\": 400, \"explain\": \"Error: Invalid box number\"}");
            }
            break;
          }
        default:
          Serial.println("{\"code\": 400, \"explain\": \"Error: Unknown command\"}");
          break;
      }

      // Clear the inputBuffer ready for the next command
      bufferIndex = 0;
    } else if (bufferIndex < sizeof(inputBuffer) - 1) {
      // Append the read character to the input buffer
      inputBuffer[bufferIndex++] = inChar;
    }
  }

  for (int i = 0; i < NUMBER_OF_BOXES; i++) {
    buttons[i].reading = digitalRead(buttons[i].pin);

    if (buttons[i].reading != buttons[i].lastState) {
      // reset the debouncing timer
      buttons[i].lastDebounceTime = millis();
    }

    // Check if debounce delay has passed
    if ((millis() - buttons[i].lastDebounceTime) > debounceDelay) {
      if (buttons[i].reading != buttons[i].state) {
        buttons[i].state = buttons[i].reading;
        // If the button is pressed and the box is closed...
        if ((buttons[i].state == 1) && (boxes[i].boxState < 0)) {
          Serial.print("{\"code\": 201, \"explain\": \"Success: Manually opened box " + String(i + 1) + "\"}");  // print a message to the serial monitor
          boxes[i].servo.write(RELEASE_ANGLE);      // open the box
          boxes[i].boxState = 1;        // change the state of the box
          buttons[i].lastDebounceTime = millis();  // update the debounce timer
        }
        // If the button is pressed and the box is open...
        else if ((buttons[i].state == 1) && (boxes[i].boxState > 0)) {
          Serial.print("{\"code\": 201, \"explain\": \"Success: Manually closed box " + String(i + 1) + "\"}");  // print a message to the serial monitor
          boxes[i].servo.write(STARTUP_ANGLE);     // close the box
          boxes[i].boxState = -1;       // change the state of the box
          buttons[i].lastDebounceTime = millis();  // update the debounce timer
        }
      }
    }
    buttons[i].lastState = buttons[i].reading;
  }
}
