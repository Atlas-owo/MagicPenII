// ----------------- Pin Definitions -----------------
const uint8_t M1_PIN     = 6;   // H-bridge input 1
const uint8_t M2_PIN     = 7;   // H-bridge input 2
const uint8_t ENA        = 5;   // PWM control

const uint8_t ENC_A_PIN  = 2;   // Encoder Channel A (interrupt 0)
const uint8_t ENC_B_PIN  = 3;   // Encoder Channel B (interrupt 1)
const uint8_t PRESSURE_SENSOR = A1; // Pressure sensor analog input

const uint8_t BUTTON_Home = 10;   
const uint8_t BUTTON_CONTROL = 11;   

// ----------------- Encoder -----------------
volatile long encoderCount = 0;

// ----------------- Serial Control -----------------
// enum MotorCommand {
//   CMD_NONE = 0,
//   CMD_FORWARD,
//   CMD_REVERSE,
//   CMD_STOP
// };

//volatile MotorCommand serialCommand = CMD_NONE;
unsigned long lastSerialCommandTime = 0;
const unsigned long SERIAL_TIMEOUT = 100; // Timeout in milliseconds
int pwmValue = 0;  // PWM duty cycle (0~255)

// ----------------- Setup -----------------
void setup() {
  // Pin modes
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(PRESSURE_SENSOR, INPUT);

  // Initial motor state
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, 0);

  // Serial
  Serial.begin(115200);
  //Serial.println("Motor Control Ready.");
  //Serial.println("Commands: Fxxx (forward), Rxxx (reverse), S (stop)");

  // Zeroing
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);
  delay(50);
  
  while (digitalRead(BUTTON_Home) != LOW) {
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
    analogWrite(ENA, 180);
    delay(10);
  }

  // Encoder
  encoderCount = 0;
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderA, RISING);
}

// ----------------- Main Loop -----------------
void loop() {
  handleSerialInput();

  if (serialCommand != CMD_NONE && millis() - lastSerialCommandTime > SERIAL_TIMEOUT) {
    serialCommand = CMD_NONE;
    stopMotor();
    Serial.println("Serial command timeout → Motor stopped");
  }

  int pressureReading = analogRead(PRESSURE_SENSOR);
  Serial.print("P");
  Serial.print(pressureReading);
  Serial.print(" | E");
  Serial.println(encoderCount);

  delay(5);  // Adjust as needed
}

// ----------------- Serial Input Handler -----------------
void handleSerialInput() {
  static String inputString = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        parseCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}

void parseCommand(const String& cmd) {
  char direction = toupper(cmd.charAt(0));
  if (direction == 'S') {
    serialCommand = CMD_STOP;
    lastSerialCommandTime = millis();
    stopMotor();
    Serial.println("Motor stopped");
    return;
  }

  int val = cmd.substring(1).toInt();
  pwmValue = constrain(val, 0, 255);

  if (direction == 'F') {
    serialCommand = CMD_FORWARD;
    lastSerialCommandTime = millis();
    forwardMotor(pwmValue);
  } else if (direction == 'R') {
    serialCommand = CMD_REVERSE;
    lastSerialCommandTime = millis();
    reverseMotor(pwmValue);
  } else {
    //Serial.println("Invalid command. Use Fxxx, Rxxx, or S.");
  }
}

// ----------------- Motor Control -----------------
void forwardMotor(int pwm) {
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, HIGH);
  analogWrite(ENA, pwm);
  Serial.print("Motor → Forward | PWM = ");
  Serial.println(pwm);
}

void reverseMotor(int pwm) {
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, pwm);
  Serial.print("Motor → Reverse | PWM = ");
  Serial.println(pwm);
}

void stopMotor() {
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, 0);
}

// ----------------- Encoder ISR -----------------
void handleEncoderA() {
  bool A = digitalRead(ENC_A_PIN);
  bool B = digitalRead(ENC_B_PIN);
  encoderCount += (A ^ B) ? -1 : +1;
}
