// Continuous Position Control - Responsive to Real-time Commands
// Motor continuously moves toward current target position
#include <Arduino.h>
// ----------------- Pin Definitions -----------------
const uint8_t M1_PIN     = 8;   // H-bridge input 1
const uint8_t M2_PIN     = 7;   // H-bridge input 2
const uint8_t ENA        = 9;   // PWM control

const uint8_t ENC_A_PIN  = 3;   // Encoder Channel A (interrupt 0)
const uint8_t ENC_B_PIN  = 2;   // Encoder Channel B (interrupt 1)
bool lastA = 0;
bool lastB = 0;
const uint8_t PRESSURE_SENSOR = A1; // Pressure sensor analog input

const uint8_t BUTTON_Home = 11;   
const uint8_t BUTTON_CONTROL = 10;   

// ----------------- Global Variables -----------------
//volatile long encoderCount = 0;
volatile long targetPosition = 0;  // Made volatile since it can change anytime

// Movement parameters
const int FAST_SPEED = 255;      // PWM for fast movement
const int SLOW_SPEED = 150;      // PWM for slow approach
const int CREEP_SPEED = 80;      // PWM for final approach

const int SLOW_DISTANCE = 300;   // Start slowing down at this distance
const int CREEP_DISTANCE = 200;   // Start creeping at this distance
const int STOP_TOLERANCE = 10;    // Stop when within this many counts

// Debugging
bool debugMode = true;
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 200; // Print debug every 200ms

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Send every 50ms

// Command parsing
String inputBuffer = "";
bool commandReady = false;

// Movement limits
const long MIN_POSITION = 50;    // Minimum allowed position (encoder counts)
const long MAX_POSITION = 10000;  // Maximum allowed position (encoder counts)

// Distance conversion constants
const float DISTANCE_SLOPE = 0.0084;  // mm per encoder count
// const float DISTANCE_OFFSET = 2.062;  // mm offset
const float DISTANCE_OFFSET = 0.5;  // mm offset
const float MIN_DISTANCE_MM = 0;    // Minimum distance in mm
const float MAX_DISTANCE_MM = 90.0;   // Maximum distance in mm
bool isHoming = false;


// Encoder variables
volatile long encoderCount = 0;
volatile uint8_t lastEncoded = 0;

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  Serial.println("=== Continuous Position Control ===");
  Serial.println("Commands:");
  Serial.println("  M<distance> - Move pen to extend distance in mm (e.g., M30.2)");
  Serial.println("  H - Home motor");
  Serial.println("  S - Stop motor (set target to current position)");
  Serial.println("  T - Test motor");
  Serial.println("  E - Test encoder");
  Serial.println("  D - Toggle debug mode");
  Serial.println("  P - Print current position and distance");
  Serial.println();

  // Pin setup
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_Home, INPUT_PULLUP);
  pinMode(BUTTON_CONTROL, INPUT_PULLUP);
  
  // Initialize motor stopped
  stopMotor();
  homeMotor();
  
  // Set initial target to current position (stopped)
  targetPosition = encoderCount;
  
  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), updateEncoder, CHANGE);

  Serial.println("Setup complete. Motor will continuously track target position.");
  Serial.print("Current position: ");
  Serial.println(encoderCount);
  
  // Reserve string buffer for commands
  inputBuffer.reserve(50);
}

// ----------------- Main Loop -----------------
void loop() {
  // ALWAYS check for and handle serial commands (non-blocking)
  handleSerialInput();
  
  // ALWAYS update motor movement toward current target
  updateContinuousMovement();

  // Send sensor data periodically
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    readSensorData();
    lastSendTime = currentTime;
  }
  
  // Debug output
  if (debugMode && (millis() - lastDebugTime > DEBUG_INTERVAL)) {
    if (abs(targetPosition - encoderCount) > STOP_TOLERANCE) {
      printMovementStatus();
    }
    lastDebugTime = millis();
  }
}

void readSensorData() {
  int analogValue = analogRead(A1);        // pressure
  int digitalValue = digitalRead(11);      // button
  int homeValue = digitalRead(10);      // button  
  float currentDistance = encoderCountsToDistance(encoderCount);
  // Send formatted string to serial
  Serial.print("P");
  Serial.print(analogValue);
  Serial.print("|E");
  Serial.print(encoderCount);
  Serial.print("|D");
  Serial.print(currentDistance, 1);
  Serial.print("|B");
  Serial.println(digitalValue);
  Serial.print("|H");
  Serial.println(homeValue);
}

// ----------------- Continuous Movement Control -----------------
void updateContinuousMovement() {



  long currentPos = encoderCount;

  // Safety check: stop if we've exceeded limits (but not during homing)
  if (!isHoming) {
    if (currentPos < MIN_POSITION && targetPosition >= MIN_POSITION) {
      // We're below minimum and trying to go up - that's OK
    } else if (currentPos > MAX_POSITION && targetPosition <= MAX_POSITION) {
      // We're above maximum and trying to go down - that's OK
    } else if (currentPos < MIN_POSITION && targetPosition < MIN_POSITION) {
      // Below minimum and target is also below - stop
      targetPosition = MIN_POSITION;
    } else if (currentPos > MAX_POSITION && targetPosition > MAX_POSITION) {
      // Above maximum and target is also above - stop
      targetPosition = MAX_POSITION;
    }
  }


  long error = targetPosition - currentPos;
  long absError = abs(error);
  
  // Check if we're close enough to target (within tolerance)
  if (absError <= STOP_TOLERANCE) {
    stopMotor();
    return;
  }
  
  // Determine direction (1 = forward, -1 = reverse)
  int direction = (error > 0) ? 1 : -1;
  
  // Determine speed based on distance to target
  int speed = FAST_SPEED;
  if (absError <= CREEP_DISTANCE) {
    speed = CREEP_SPEED;
  } else if (absError <= SLOW_DISTANCE) {
    speed = SLOW_SPEED;
  }
  
  // Apply movement
  setMotorDirection(direction);
  setMotorSpeed(speed);
}

// Distance conversion functions
long distanceToEncoderCounts(float distanceMM) {
  return (long)((distanceMM - DISTANCE_OFFSET) / DISTANCE_SLOPE);
}

float encoderCountsToDistance(long counts) {
  return (counts * DISTANCE_SLOPE) + DISTANCE_OFFSET;
}

void setTargetDistance(float distanceMM) {
  if (distanceMM < MIN_DISTANCE_MM) {
    distanceMM = MIN_DISTANCE_MM;
    Serial.print("Distance clamped to minimum: ");
  } else if (distanceMM > MAX_DISTANCE_MM) {
    distanceMM = MAX_DISTANCE_MM;
    Serial.print("Distance clamped to maximum: ");
  }
  
  long targetCounts = distanceToEncoderCounts(distanceMM);
  setTargetPosition(targetCounts);
  
  Serial.print("Target distance: ");
  Serial.print(distanceMM, 1);
  Serial.print("mm (counts: ");
  Serial.print(targetCounts);
  Serial.println(")");
}

void setTargetPosition(long target) {
  if (!isHoming) {
    if (target < MIN_POSITION) {
      target = MIN_POSITION;
      //Serial.print("Target clamped to minimum: ");
    } else if (target > MAX_POSITION) {
      target = MAX_POSITION;
      //Serial.print("Target clamped to maximum: ");
    } else {
      //Serial.print("New target: ");
    }
  }

  // Immediately update target - motor will start moving toward it on next loop
  targetPosition = target;
  
  Serial.print("New target: ");
  Serial.print(target);
  Serial.print(" (current: ");
  Serial.print(encoderCount);
  Serial.print(", distance: ");
  Serial.print(abs(target - encoderCount));
  Serial.println(")");
}

void setMotorDirection(int direction) {
  if (direction > 0) {
    // Forward
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, HIGH);
  } else {
    // Reverse
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, LOW);
  }
}

void setMotorSpeed(int speed) {
  analogWrite(ENA, speed);
}

void stopMotor() {
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  analogWrite(ENA, 0);
}

void stopMovement() {
  // Set target to current position to stop movement
  targetPosition = encoderCount;
  stopMotor();
  Serial.println("Movement stopped. Target set to current position.");
}

// ----------------- Non-blocking Serial Input Handler -----------------
void handleSerialInput() {
  // Read available characters without blocking
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";  // Clear buffer
      }
    } else {
      inputBuffer += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 'M': {
      if (command.length() > 1) {
        float distanceMM = command.substring(1).toFloat();
        if (distanceMM > 0) {
          setTargetDistance(distanceMM);
        } else {
          Serial.println("Invalid distance. Must be positive.");
        }
      } else {
        Serial.println("Usage: M<distance> (e.g., M30.2 for 30.2mm)");
      }
      break;
    }
    
    case 'H':
      homeMotor();
      break;
      
    case 'S':
      stopMovement();
      break;
      
    case 'T':
      testMotor();
      break;
      
    case 'E':
      testEncoder();
      break;
      
    case 'D':
      debugMode = !debugMode;
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "ON" : "OFF");
      break;
      
    case 'P': {
      float currentDistance = encoderCountsToDistance(encoderCount);
      float targetDistance = encoderCountsToDistance(targetPosition);
      Serial.print("Position: ");
      Serial.print(encoderCount);
      Serial.print(" counts (");
      Serial.print(currentDistance, 1);
      Serial.print("mm), Target: ");
      Serial.print(targetPosition);
      Serial.print(" counts (");
      Serial.print(targetDistance, 1);
      Serial.print("mm), Error: ");
      Serial.println(targetPosition - encoderCount);
      break;
    }
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      Serial.println("Use H, M<distance>, S, T, E, D, or P");
  }
}

// ----------------- Homing -----------------
// ----------------- Modified Homing Function -----------------
void homeMotor() {
  Serial.println("Homing motor...");
  isHoming = true;
  
  // Set target far in reverse direction to start homing movement
  targetPosition = -10000;  // This will make it move toward home
  
  // Wait for button press while allowing continuous movement
  while (digitalRead(BUTTON_Home) == HIGH) {
    updateContinuousMovement();  // Keep moving during homing
    delay(10);
    
    // Safety check - print position occasionally
    static unsigned long lastHomePrint = 0;
    if (millis() - lastHomePrint > 500) {
      Serial.print("Homing... position: ");
      Serial.println(encoderCount);
      lastHomePrint = millis();
    }
  }
  
  // Button pressed - continue moving for 0.5 seconds
  Serial.println("Home button triggered. Continuing for 0.5 seconds...");
  
  // Keep motor moving in the same direction (reverse) for 500ms
  // We manually control the motor here instead of using updateContinuousMovement
  setMotorDirection(-1);  // Reverse direction
  setMotorSpeed(CREEP_SPEED);  // Use slow speed for final approach
  
  unsigned long continueStartTime = millis();
  while (millis() - continueStartTime < 1000) {
    // Just keep the motor running for 850ms
    delay(10);
    
    // Optional: print position during this final movement
    static unsigned long lastContinuePrint = 0;
    if (millis() - lastContinuePrint > 100) {
      Serial.print("Final positioning... encoder: ");
      Serial.println(encoderCount);
      lastContinuePrint = millis();
    }
  }
  
  // Stop motor after 0.5 seconds
  stopMotor();
  delay(100);
  
  // Reset encoder and target
  encoderCount = 0;
  targetPosition = 0;  // Start at home position
  isHoming = false;
  
  Serial.println("Homing complete. Position reset to 0.");
  Serial.println("Ready for operation.");
}

// ----------------- Test Functions -----------------
void testMotor() {
  Serial.println("=== Motor Test ===");
  
  Serial.println("Moving to 10mm...");
  setTargetDistance(10.0);
  delay(3000);  // Let it move for 3 seconds
  
  Serial.println("Moving to 25mm...");
  setTargetDistance(25.0);
  delay(3000);  // Let it move for 3 seconds
  
  Serial.println("Returning to 5mm...");
  setTargetDistance(5.0);
  delay(2000);
  
  Serial.println("Motor test complete.");
  float finalDistance = encoderCountsToDistance(encoderCount);
  Serial.print("Final position: ");
  Serial.print(encoderCount);
  Serial.print(" counts (");
  Serial.print(finalDistance, 1);
  Serial.println("mm)");
}

void testEncoder() {
  Serial.println("=== Encoder Test ===");
  Serial.println("Manually turn the motor shaft...");
  Serial.println("Watching for 10 seconds...");
  
  long startCount = encoderCount;
  unsigned long startTime = millis();
  long lastPrintedCount = startCount;
  
  // Stop motor movement during encoder test
  long savedTarget = targetPosition;
  targetPosition = encoderCount;
  
  while (millis() - startTime < 10000) {
    if (encoderCount != lastPrintedCount) {
      Serial.print("Encoder: ");
      Serial.print(encoderCount);
      Serial.print(" (change: ");
      Serial.print(encoderCount - lastPrintedCount);
      Serial.println(")");
      lastPrintedCount = encoderCount;
    }
    delay(50);
  }
  
  // Restore target
  targetPosition = savedTarget;
  
  Serial.print("Encoder test complete. Total change: ");
  Serial.println(encoderCount - startCount);
}

// ----------------- Debug Functions -----------------
void printMovementStatus() {
  long error = targetPosition - encoderCount;
  Serial.print("Pos: ");
  Serial.print(encoderCount);
  Serial.print(" | Target: ");
  Serial.print(targetPosition);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Distance: ");
  Serial.println(abs(error));
}

// ----------------- Encoder ISR -----------------
void updateEncoder() {
  // State table for quadrature decoding
  // Lookup table based on previous and current AB states
  static const int8_t encoderStates[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  
  // Read current state of both pins
  uint8_t a = digitalRead(ENC_A_PIN);
  uint8_t b = digitalRead(ENC_B_PIN);
  
  // Create 2-bit current state
  uint8_t currentEncoded = (a << 1) | b;
  
  // Combine with last state to create 4-bit lookup value
  uint8_t sum = (lastEncoded << 2) | currentEncoded;
  
  // Update count based on state transition
  encoderCount += encoderStates[sum];
  
  // Save current state for next time
  lastEncoded = currentEncoded;
}


// ----------------- Additional Helper Functions -----------------
void emergencyStop() {
  stopMotor();
  targetPosition = encoderCount;  // Set target to current position
  Serial.println("EMERGENCY STOP!");
}

void printStatus() {
  float currentDistance = encoderCountsToDistance(encoderCount);
  float targetDistance = encoderCountsToDistance(targetPosition);
  
  Serial.println("=== Status ===");
  Serial.print("Position: ");
  Serial.print(encoderCount);
  Serial.print(" counts (");
  Serial.print(currentDistance, 1);
  Serial.println("mm)");
  Serial.print("Target: ");
  Serial.print(targetPosition);
  Serial.print(" counts (");
  Serial.print(targetDistance, 1);
  Serial.println("mm)");
  Serial.print("Error: ");
  Serial.print(targetPosition - encoderCount);
  Serial.print(" counts (");
  Serial.print(targetDistance - currentDistance, 1);
  Serial.println("mm)");
  Serial.print("Moving: ");
  Serial.println(abs(targetPosition - encoderCount) > STOP_TOLERANCE ? "YES" : "NO");
  Serial.print("Debug: ");
  Serial.println(debugMode ? "ON" : "OFF");
  Serial.println("===============");
}