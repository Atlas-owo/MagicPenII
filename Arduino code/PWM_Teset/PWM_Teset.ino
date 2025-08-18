const uint8_t M1_PIN = 6;   // H-bridge input 1
const uint8_t M2_PIN = 7;   // H-bridge input 2
const uint8_t ENA    = 5;   // PWM 控制引脚

void setup() {
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);

  // 初始停止
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, 0);

  Serial.begin(115200);
  Serial.println("PWM Motor Test Ready.");
  Serial.println("Commands: Fxxx (forward), Rxxx (reverse), S (stop), e.g., F150");
}

void loop() {
  handleSerialInput();
  
  delay(100);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, 0);
}

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
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
    analogWrite(ENA, 0);
    Serial.println("Motor Stopped");
    return;
  }

  int pwmValue = cmd.substring(1).toInt();
  pwmValue = constrain(pwmValue, 0, 255);  // 限制 PWM 范围

  if (direction == 'F') {
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, HIGH);
    analogWrite(ENA, pwmValue);
    Serial.print("Motor Forward, PWM = ");
    Serial.println(pwmValue);
  } else if (direction == 'R') {
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
    analogWrite(ENA, pwmValue);
    Serial.print("Motor Reverse, PWM = ");
    Serial.println(pwmValue);
  } else {
    Serial.println("Invalid Command. Use Fxxx, Rxxx, or S.");
  }

}
