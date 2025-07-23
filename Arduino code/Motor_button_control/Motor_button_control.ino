// Pin assignments
const uint8_t BUTTON_FWD = 9;   // Forward button, pulls to GND when pressed
const uint8_t BUTTON_REV = 8;   // Reverse button, pulls to GND when pressed
const uint8_t M1_PIN     = 6;   // H-bridge input 1
const uint8_t M2_PIN     = 7;   // H-bridge input 2
const uint8_t ENC_A_PIN = 2;   // Encoder Channel A (interrupt 0)
const uint8_t ENC_B_PIN = 3;   // Encoder Channel B (interrupt 1)
const uint8_t PRESSURE_SENSOR     = A1;   // Pressure sensor
const uint8_t ENA       = 5;   // PWM control
const uint8_t BUTTON_Home_DETECT      =   10;   
const uint8_t BUTTON_Home_CONTROL     =   11;   

// --------- Encoder & Control ---------
volatile long encoderCount = 0;

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

  // Zeroing
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);
  delay(50);
  
  // while (digitalRead(BUTTON_Home_DETECT) != LOW) {
  //   digitalWrite(M1_PIN, HIGH);
  //   digitalWrite(M2_PIN, LOW);
  //   analogWrite(ENA, 180);
  //   delay(10);
  // }

  encoderCount = 0;
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderA, RISING);
  Serial.println("Zeroing done.");
  
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
  Serial.print("pressure ");
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

    if (encoderCount < 400) {
    // Forward: M1=HIGH, M2=LOW
    //analogWrite(ENA, 200);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, HIGH);  
    Serial.println("Motor → Forward");
    }
    else {
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);      
    }
    //analogWrite(ENA, 200);

  }

  else if (reversePressed && !forwardPressed) {
    if (encoderCount > 50) {
    // Reverse: M1=LOW, M2=HIGH
    //analogWrite(ENA, 200);
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
    }
    else {
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);      
    }

    //Serial.println("Motor → Reverse");
  }
  
  else if (!reversePressed && !forwardPressed){
    // Neither or both pressed: coast (both LOW)
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
    //Serial.println("Motor → Stopped");

    if (pressureReading > 10 && encoderCount > 100) {
    // analogWrite(ENA, 170);
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
    //Serial.println("Pressure too high, motor → Reverse");
    }
  }

  // else if (pressureReading > 100) {
    
  //   //forwardPressed = true;  // Force the motor to reverse if pressure is too high
  //   digitalWrite(M1_PIN, HIGH);
  //   digitalWrite(M2_PIN, LOW);
  //   Serial.println("Pressure too high, motor → Reverse");
  // }
  Serial.println(encoderCount);
  delay(10);  // simple debounce & status update interval
}


void handleEncoderA() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  encoderCount += (A ^ B) ? -1 : +1;
  //Serial.print(encoderCount);
  // Serial.print(" ");
  // Serial.print(A);
  // Serial.print(" ");
  // Serial.print(B);
  //Serial.println(" ");
}
