// --------- Pin Definitions ---------
const uint8_t ENC_A_PIN = 2;   // Encoder Channel A (interrupt 0)
const uint8_t ENC_B_PIN = 3;   // Encoder Channel B (interrupt 1)
const uint8_t M1_PIN    = 6;   // H-bridge input 1
const uint8_t M2_PIN    = 7;   // H-bridge input 2
const uint8_t ENA       = 5;   // PWM control

// --------- Encoder & Control ---------
volatile long encoderCount = 0;
long targetCount   = 0;
unsigned long lastTime = 0;

// How close (in counts) before we stop
const long deadband = 21;

// --------- PID Parameters ---------
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.1;

float integral = 0;
long prevError = 0;

// --------- Setup ---------
void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Start stopped
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderA, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), handleEncoderB, RISING);

  Serial.begin(115200);
  lastTime = millis();
}

// --------- Main Loop ---------
void loop() {
  // Set new target with “move X” (relative) over serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("move ")) {
      long steps = cmd.substring(5).toInt();
      targetCount = encoderCount + steps;
      integral    = 0;
      prevError   = 0;
    }
  }

  // Control at ~50 Hz
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // seconds
  if (now - lastTime >= 20) {
    long error = targetCount - encoderCount;
    // Serial.print(targetCount);
    // Serial.print(" ");
    // Serial.print(encoderCount);
    // Serial.print(" ");
    // Serial.print(error);
    // Serial.println(" ");

    integral += error * dt;
    float derivative = (error - prevError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Convert to 0–255 PWM magnitude
    int pwm = constrain(abs(output), 0, 255);
    Serial.print(pwm);
    Serial.print(".  ");
    Serial.print(encoderCount);
    Serial.println(".  ");

    if(pwm > 40){
      analogWrite(ENA, pwm);
    }
    else{
      analogWrite(ENA, 0);
    }
    if (error > deadband) {
         
      digitalWrite(M1_PIN, HIGH);
      digitalWrite(M2_PIN, LOW);
    }
    else if (error < -deadband) {

      digitalWrite(M1_PIN, LOW);
      digitalWrite(M2_PIN, HIGH);
      
    }
    else {
      // Within deadband → stop (coast)
      digitalWrite(M1_PIN, LOW);
      digitalWrite(M2_PIN, LOW);
    }
    prevError  = error;
    lastTime = now;
  }
}

// --------- Encoder ISRs (quadrature) ---------
void handleEncoderA() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  encoderCount += (A ^ B) ? +1 : -1;
  //Serial.print(encoderCount);
  // Serial.print(" ");
  // Serial.print(A);
  // Serial.print(" ");
  // Serial.print(B);
  //Serial.println(" ");
}
