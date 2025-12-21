#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =================================================================================
// COMPETITIVE LINE FOLLOWER & MAZE SOLVER (2-Button Version)
// =================================================================================

// --- OLED Display Settings ---
// We use the Adafruit SSD1306 library for the I2C OLED display.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- QTR-8RC Sensor Settings ---
// The QTR-8RC sensor array uses 8 IR sensors to detect the line.
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
// Sensors connected to digital pins D2 through D9
const uint8_t sensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

// --- Motor Driver (TB6612FNG) Pin Definitions ---
// The TB6612FNG requires a PWM pin for speed and two digital pins for direction per motor.
#define PWMA 10  // Left Motor Speed (PWM capable pin)
#define AIN1 A0  // Left Motor Direction 1
#define AIN2 A1  // Left Motor Direction 2
#define PWMB 11  // Right Motor Speed (PWM capable pin)
#define BIN1 A2  // Right Motor Direction 1
#define BIN2 A3  // Right Motor Direction 2

// --- Other Hardware Pins ---
#define LED_PIN 12      // White LED to indicate "End Box" detection

// --- NEW BUTTON PINS (Discrete Analog Inputs) ---
// Note: A6 and A7 on the Nano are ANALOG INPUT ONLY. They cannot be digital outputs.
// We use external 10k pull-down resistors.
// Unpressed = 0V (Read ~0), Pressed = 5V (Read ~1023).
#define BTN_NEXT_PIN   A7  // Button to scroll menu options
#define BTN_SELECT_PIN A6  // Button to confirm selection

// --- PID Control Constants (TUNE THESE ON TRACK!) ---
// PID (Proportional-Integral-Derivative) keeps the bot centered on the line.
float Kp = 0.1;   // Proportional: Reacts to current error. (Primary tuning value)
float Ki = 0.00;  // Integral: Reacts to accumulated error. (Usually 0 for line followers)
float Kd = 0.5;   // Derivative: Reacts to rate of change. (Dampens oscillation)

// --- PID Variables ---
int lastError = 0;
int integral = 0;
// The QTR library returns 0-7000 for 8 sensors. Center is 3500.
int target_position = 3500; 

// --- Motor Speed Settings ---
const int MAX_SPEED = 200;  // Maximum PWM limit (0-255)
const int BASE_SPEED = 150; // Cruising speed on straight lines
const int TURN_SPEED = 120; // Speed while performing sharp turns

// --- Maze Solving Variables ---
// We store the path as a string of characters: 'L' (Left), 'R' (Right), 'S' (Straight), 'B' (Back)
char path[100];
int pathLength = 0;
int pathIndex = 0;

// --- Game Timing Variables ---
unsigned long startTime;
unsigned long dryRunDuration = 0;
unsigned long actualRunTimeLimit = 0;
// Standard competition timing rules
const unsigned long DRY_RUN_LIMIT_MS = 3 * 60 * 1000UL;       // 3 minutes
const unsigned long BASE_ACTUAL_LIMIT_MS = 2 * 60 * 1000UL + 30 * 1000UL; // 2m 30s

// --- Menu States ---
enum MenuState { MAIN_MENU, CALIBRATING, DRY_RUN, ACTUAL_RUN, COMPLETED };
MenuState currentState = MAIN_MENU;
int menuOption = 0; // 0: Calibrate, 1: Dry Run, 2: Actual Run

// --- Function Prototypes ---
void setupHardware();
void handleMenu();
void runCalibration();
void doDryRun();
void doActualRun();
void runPID(uint16_t position);
void handleIntersection(bool isDryRun);
void simplifyPath();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void turnLeft();
void turnRight();
void turnBack();
void stopMotors();
int readButtons();
void displayUpdate(String line1, String line2 = "", String line3 = "", String line4 = "");

// =================================================================================
// MAIN SETUP & LOOP
// =================================================================================

void setup() {
  setupHardware();
  displayUpdate("Bot Ready!", "Select Mode", "Use Buttons");
  delay(2000);
}

void loop() {
  // Main State Machine
  switch (currentState) {
    case MAIN_MENU:
      handleMenu();
      break;
      
    case CALIBRATING:
      runCalibration();
      currentState = MAIN_MENU; // Return to menu after calibration
      break;
      
    case DRY_RUN:
      doDryRun();
      currentState = MAIN_MENU; // Return to menu after run
      break;
      
    case ACTUAL_RUN:
      doActualRun();
      currentState = MAIN_MENU; // Return to menu after run
      break;
      
    case COMPLETED:
       // Wait for user interaction to reset to main menu
       if(readButtons() != 0) {
         delay(500);
         currentState = MAIN_MENU;
       }
       break;
  }
}

// =================================================================================
// HARDWARE & MENU FUNCTIONS
// =================================================================================

void setupHardware() {
  Serial.begin(9600);
  
  // Initialize OLED Display (Address 0x3C)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Loop forever if display fails
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize QTR Sensors
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  // Initialize Motor Pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Note: A6 and A7 do not need pinMode setup on Arduino Nano.
}

// --- UPDATED BUTTON READING LOGIC ---
// Returns: 0 (None), 1 (Next), 2 (Select)
int readButtons() {
  // A6/A7 are 10-bit analog inputs (0-1023).
  // With a 10k pull-down resistor:
  // - Button OPEN: 0V -> Analog Read 0
  // - Button CLOSED: 5V -> Analog Read ~1023
  // We use a threshold of 500 to clearly distinguish HIGH from LOW.
  
  if (analogRead(BTN_SELECT_PIN) > 500) {
    return 2; // Select Button Pressed
  }
  if (analogRead(BTN_NEXT_PIN) > 500) {
    return 1; // Next Button Pressed
  }
  
  return 0; // No button pressed
}

void handleMenu() {
  int button = readButtons();
  delay(200); // Simple debounce delay

  // Button 1 (Next): Cycle through menu options
  if (button == 1) { 
    menuOption = (menuOption + 1) % 3; // Cycles 0 -> 1 -> 2 -> 0...
  }

  // Draw the Menu UI
  String opt1 = (menuOption == 0) ? "> 1. Calibrate" : "  1. Calibrate";
  String opt2 = (menuOption == 1) ? "> 2. Dry Run" : "  2. Dry Run";
  String opt3 = (menuOption == 2) ? "> 3. Actual Run" : "  3. Actual Run";
  displayUpdate("MAIN MENU", opt1, opt2, opt3);

  // Button 2 (Select): Confirm choice
  if (button == 2) { 
    delay(500); // Wait for button release
    if (menuOption == 0) currentState = CALIBRATING;
    if (menuOption == 1) currentState = DRY_RUN;
    if (menuOption == 2) currentState = ACTUAL_RUN;
  }
}

// Helper to update OLED with up to 4 lines of text
void displayUpdate(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  display.setCursor(0, 0); display.println(line1);
  display.setCursor(0, 16); display.println(line2);
  display.setCursor(0, 32); display.println(line3);
  display.setCursor(0, 48); display.println(line4);
  display.display();
}

// =================================================================================
// CORE LOGIC: CALIBRATION & RUN MODES
// =================================================================================

void runCalibration() {
  displayUpdate("Calibrating...", "Spin bot over", "line & dark area.");
  
  // Rotate bot back and forth to expose sensors to min (white) and max (black) values
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    // Spin in place (Left motor forward, Right motor backward)
    setMotorSpeeds(100, -100); 
  }
  stopMotors();
  
  // Wait for user confirmation before returning
  displayUpdate("Calibration", "Complete!", "Press SELECT");
  while(readButtons() != 2) delay(10);
  delay(500);
}

void doDryRun() {
  displayUpdate("DRY RUNNING...", "Path finding ON");
  startTime = millis();
  pathLength = 0;
  digitalWrite(LED_PIN, LOW);

  while (true) {
    // 1. READ SENSORS: Get line position (0-7000)
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // 2. CHECK FOR END ZONE (All sensors see White)
    int whiteSensorCount = 0;
    for(int i=0; i<SensorCount; i++) {
        if(sensorValues[i] < 200) whiteSensorCount++;
    }

    if (whiteSensorCount >= 6) { 
      // End Zone Detected
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      dryRunDuration = millis() - startTime;
      
      // Calculate Time Penalty (if dry run took > 3 mins)
      unsigned long penalty = 0;
      if (dryRunDuration > DRY_RUN_LIMIT_MS) {
        penalty = dryRunDuration - DRY_RUN_LIMIT_MS;
      }
      
      // Calculate allowed time for Actual Run
      if (penalty > BASE_ACTUAL_LIMIT_MS) {
        actualRunTimeLimit = 0;
      } else {
        actualRunTimeLimit = BASE_ACTUAL_LIMIT_MS - penalty;
      }

      displayUpdate("DRY RUN DONE!", "Time: " + String(dryRunDuration/1000) + "s", 
                    "Limit: " + String(actualRunTimeLimit/1000) + "s");
      
      // Simplify the path (remove U-turns)
      simplifyPath(); 
      delay(3000);
      currentState = COMPLETED;
      return;
    }

    // 3. CHECK FOR INTERSECTION PATTERNS
    // Threshold < 400 usually means White Line detected
    const int THRESHOLD = 400;
    bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
    bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);

    // 4. DECISION: INTERSECTION OR PID?
    if (leftDetect || rightDetect) {
       // Intersection found: Hand over to intersection logic
       handleIntersection(true); // true = recording mode
       
       // Reset PID memory to prevent "jerking" after a turn
       lastError = 0;
       integral = 0;
    } else {
       // No intersection: Just follow the line
       runPID(position);
    }
  }
}

void doActualRun() {
  // Safety check: Don't run if no path recorded
  if (pathLength == 0) {
    displayUpdate("ERROR: No Path!", "Perform Dry Run", "First.");
    delay(2000);
    return;
  }
  
  displayUpdate("ACTUAL RUNNING...", "Speed path ON");
  startTime = millis();
  pathIndex = 0;
  digitalWrite(LED_PIN, LOW);

  while (true) {
    // 1. READ SENSORS
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // 2. CHECK TIME LIMIT
    if (millis() - startTime > actualRunTimeLimit) {
      stopMotors();
      displayUpdate("GAME OVER", "Time Limit", "Exceeded!");
      delay(3000);
      currentState = COMPLETED;
      return;
    }
    
    // 3. CHECK END ZONE
    int whiteSensorCount = 0;
    for(int i=0; i<SensorCount; i++) {
        if(sensorValues[i] < 200) whiteSensorCount++;
    }

    if (whiteSensorCount >= 6) {
      // Finished!
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      unsigned long finalTime = millis() - startTime;
      displayUpdate("FINISHED!", "Time: " + String(finalTime/1000) + "." + String((finalTime%1000)/10) + "s");
      delay(3000);
      currentState = COMPLETED;
      return;
    }

    // 4. CHECK INTERSECTION
    const int THRESHOLD = 400;
    bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
    bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);

    // 5. DECISION
    if (leftDetect || rightDetect) {
        handleIntersection(false); // false = replay mode
        lastError = 0;
        integral = 0;
    } else {
        runPID(position);
    }
  }
}

// =================================================================================
// LINE FOLLOWING & MAZE SOLVING LOGIC
// =================================================================================

void runPID(uint16_t position) {
  // Calculate Error (How far from center 3500 are we?)
  int error = position - target_position;
  
  // Accumulate Integral (with anti-windup limits)
  integral = integral + error;
  if (integral > 10000) integral = 10000;
  if (integral < -10000) integral = -10000;
  
  // Calculate Derivative (Rate of change of error)
  int derivative = error - lastError;
  lastError = error;

  // Compute PID Output
  int output = Kp * error + Ki * integral + Kd * derivative;
  
  // Apply to Motor Speeds
  // If output is positive, turn Right (Slow down right motor, Speed up left)
  // If output is negative, turn Left
  int leftMotorSpeed = BASE_SPEED + output;
  int rightMotorSpeed = BASE_SPEED - output;

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void handleIntersection(bool isDryRun) {
  const int THRESHOLD = 400;
  
  // Check sensors just as we enter intersection
  bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
  bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);
  bool straightDetect = (sensorValues[3] < THRESHOLD) || (sensorValues[4] < THRESHOLD);

  // Filter noise
  if (!leftDetect && !rightDetect) return;

  // Inch forward blindly to center the bot wheels on the intersection
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(100); 

  // Re-read sensors at the actual center of the junction
  qtr.readLineWhite(sensorValues);
  leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
  rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);
  straightDetect = (sensorValues[3] < THRESHOLD) || (sensorValues[4] < THRESHOLD);

  // Protect path array from overflow
  if (pathLength >= 99) return;

  if (isDryRun) {
    // --- RECORDING MODE (Left Hand Rule) ---
    // Priority: Left -> Straight -> Right -> Back
    if (leftDetect) {
      path[pathLength++] = 'L';
      turnLeft();
    } else if (straightDetect) {
      path[pathLength++] = 'S';
      // Continue straight (no turn function needed)
    } else if (rightDetect) {
      path[pathLength++] = 'R';
      turnRight();
    } else {
      // Dead End
      path[pathLength++] = 'B';
      turnBack();
    }
  } else {
    // --- REPLAY MODE ---
    // Execute the next turn from the simplified path
    if (pathIndex >= pathLength) return;
    char turnToTake = path[pathIndex++];
    
    switch(turnToTake) {
      case 'L': turnLeft(); break;
      case 'S': break; // Just drive forward
      case 'R': turnRight(); break;
      // Note: 'B' should not exist in a simplified path!
    }
  }
}

// Reduces the recorded path by optimizing U-turns ('B')
// Example: "Left, Back, Left" (LBL) becomes "Straight" (S)
void simplifyPath() {
  // Only simplify if we have at least 3 turns
  if (pathLength < 3) return;

  int i = 0;
  while (i <= pathLength - 3) {
    if (path[i + 1] == 'B') {
      char newTurn = '?';
      // Substitution Rules
      if      (path[i] == 'L' && path[i+2] == 'L') newTurn = 'S';
      else if (path[i] == 'L' && path[i+2] == 'R') newTurn = 'B';
      else if (path[i] == 'L' && path[i+2] == 'S') newTurn = 'R';
      else if (path[i] == 'S' && path[i+2] == 'L') newTurn = 'R';
      else if (path[i] == 'S' && path[i+2] == 'S') newTurn = 'B';
      else if (path[i] == 'R' && path[i+2] == 'L') newTurn = 'B';

      if (newTurn != '?') {
        path[i] = newTurn;
        // Shift remaining path array down by 2 slots
        for (int j = i + 1; j < pathLength - 2; j++) {
          path[j] = path[j + 2];
        }
        pathLength -= 2;
        
        // Backtrack 'i' to check if the new turn created a new pattern
        if (i > 0) i -= 2;
        else i = -1;
      }
    }
    i++;
  }
}

// =================================================================================
// MOTOR CONTROL FUNCTIONS
// =================================================================================

// Low-level motor control for TB6612FNG
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Cap speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Left Motor Logic
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(leftSpeed));
  }

  // Right Motor Logic
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(rightSpeed));
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

// --- HARD TURNS (Sensor Based) ---

void turnLeft() {
  // 1. Start spinning in place
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  
  // 2. Blind spin for a moment to clear the current line
  delay(200); 
  
  // 3. Keep spinning until center sensors find the line again
  while(true) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break; // Line Found!
    }
  }
  stopMotors();
  delay(50); // Stabilize
}

void turnRight() {
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(200);
  while(true) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break;
    }
  }
  stopMotors();
  delay(50);
}

void turnBack() {
  // U-Turn (Usually a 180 spin to the right)
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(400); // Longer blind delay for 180 degrees
  while(true) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break;
    }
  }
  stopMotors();
  delay(50);
}