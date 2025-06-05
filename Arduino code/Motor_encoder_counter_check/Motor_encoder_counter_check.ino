// --------- Pin Definitions ---------
const uint8_t ENC_A_PIN = 2;   // Encoder Channel A (interrupt 0)
const uint8_t ENC_B_PIN = 3;   // Encoder Channel B (interrupt 1)
const uint8_t M1_PIN    = 6;   // H-bridge input 1
const uint8_t M2_PIN    = 7;   // H-bridge input 2

// --------- Encoder & Control ---------
volatile long encoderCount = 0;
long targetCount   = 0;
unsigned long lastTime = 0;

// How close (in counts) before we stop
const long deadband = 2;

void handleEncoderA() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  encoderCount += (A ^ B) ? +1 : -1;
  Serial.print(encoderCount);
  Serial.print(" ");
  Serial.print(A);
  Serial.print(" ");
  Serial.print(B);
  Serial.println(" ");
}

// --------- Setup ---------
void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

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
  
  
}

// --------- Encoder ISRs (quadrature) ---------

