


int gripper_state = 1;

int relay_1_pin = 11;
int relay_2_pin = 12;

// A buffer to hold incoming commands from the Serial input
char inputBuffer[200];
int bufferIndex = 0;

void setup() {

  pinMode(relay_1_pin, OUTPUT);
  pinMode(relay_2_pin, OUTPUT);

  digitalWrite(relay_1_pin, HIGH);
  digitalWrite(relay_2_pin, LOW);

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
          {
            digitalWrite(relay_1_pin, HIGH);
            digitalWrite(relay_2_pin, LOW);

            gripper_state = 1;
            Serial.println("{\"code\": 200, \"explain\": \"Success: Opened gripper\"}");
            break;
          }
        case 'c':
          {  // Close command

            digitalWrite(relay_1_pin, LOW);
            digitalWrite(relay_2_pin, HIGH);
            gripper_state = -1;
            Serial.println("{\"code\": 200, \"explain\": \"Success: Closed gripper\"}");
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
}
