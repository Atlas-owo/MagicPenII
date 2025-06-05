// Pin assignments
const uint8_t BUTTON_FWD = 8;   // Forward button, pulls to GND when pressed
const uint8_t BUTTON_REV = 9;   // Reverse button, pulls to GND when pressed
const uint8_t M1_PIN     = 6;   // H-bridge input 1
const uint8_t M2_PIN     = 7;   // H-bridge input 2
const uint8_t PRESSURE_SENSOR     = A1;   // Pressure sensor
const uint8_t ENA       = 5;   // PWM control
const uint8_t BUTTON_Home_DETECT      =   10;   
const uint8_t BUTTON_Home_CONTROL     =   11;   

void setup() {
  // Buttons
  pinMode(BUTTON_FWD, INPUT_PULLUP);
  pinMode(BUTTON_REV, INPUT_PULLUP);
  pinMode(BUTTON_Home_DETECT, INPUT_PULLUP);
  pinMode(BUTTON_Home_CONTROL, INPUT_PULLUP);

  // Motor outputs
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(PRESSURE_SENSOR, INPUT);

  // Ensure motor is stopped on start
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  digitalWrite(ENA, HIGH);
  Serial.begin(115200);
  Serial.println("Two-button motor control ready");
}

void loop() {
  bool forwardPressed = (digitalRead(BUTTON_FWD) == LOW);
  bool reversePressed = (digitalRead(BUTTON_REV) == LOW);
  bool homePressed = (digitalRead(BUTTON_Home_CONTROL) == LOW);
  bool homeReached = (digitalRead(BUTTON_Home_DETECT) == LOW);
  // Serial.println(homeReached);
  // Serial.println(homePressed);

  // Read pressure sensor value
  int pressureReading = analogRead(PRESSURE_SENSOR);
  Serial.println(pressureReading);
  // Check if the pressure sensor is over 150


  // if (homePressed && !homeReached){
  //   Serial.println("Homeing");
  //   while(!homeReached){
  //     digitalWrite(M1_PIN, HIGH);
  //     digitalWrite(M2_PIN, LOW);
  //     delay(100);
  //     homeReached = (digitalRead(BUTTON_Home_DETECT) == LOW);
  //   }   
    
    
  // }

  if (forwardPressed && !reversePressed) {
    // Forward: M1=HIGH, M2=LOW
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
    Serial.println("Motor → Forward");
  }
  else if (reversePressed && !forwardPressed) {
    // Reverse: M1=LOW, M2=HIGH
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, HIGH);
    Serial.println("Motor → Reverse");
  }
  
  else if (!reversePressed && !forwardPressed){
    // Neither or both pressed: coast (both LOW)
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
    //Serial.println("Motor → Stopped");
  //   if (pressureReading > 100) {
    
  //   //forwardPressed = true;  // Force the motor to reverse if pressure is too high
  //   digitalWrite(M1_PIN, HIGH);
  //   digitalWrite(M2_PIN, LOW);
  //   Serial.println("Pressure too high, motor → Reverse");
  // }
  }

  // else if (pressureReading > 100) {
    
  //   //forwardPressed = true;  // Force the motor to reverse if pressure is too high
  //   digitalWrite(M1_PIN, HIGH);
  //   digitalWrite(M2_PIN, LOW);
  //   Serial.println("Pressure too high, motor → Reverse");
  // }

  delay(100);  // simple debounce & status update interval
}