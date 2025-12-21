#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =================================================================================
// COMPETITIVE LINE FOLLOWER & MAZE SOLVER (Revised Version)
// =================================================================================

// --- OLED Display Settings ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- QTR-8RC Sensor Settings ---
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
// Sensors connected to digital pins 2-9
const uint8_t sensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

// --- Motor Driver (TB6612FNG) Pin Definitions (FIXED for Nano) ---
#define PWMA 10  // Left Motor Speed
#define AIN1 A0  // Left Motor Dir 1
#define AIN2 A1  // Left Motor Dir 2
#define PWMB 11  // Right Motor Speed
#define BIN1 A2  // Right Motor Dir 1
#define BIN2 A3  // Right Motor Dir 2

// --- Other Hardware Pins ---
#define LED_PIN 12      // White LED for "End Box"
#define BUTTON_PIN A6   // Analog pin for 3-button ladder

// --- PID Control Constants (TUNE THESE ON TRACK!) ---
float Kp = 0.1;   // Proportional (Start small, e.g., 0.1 - 0.5)
float Ki = 0.00;  // Integral (Usually 0 for line followers)
float Kd = 0.5;   // Derivative (Start larger than Kp, e.g., 1.0 - 5.0)

// --- PID Variables ---
int lastError = 0;
int integral = 0;
int target_position = 3500; // Center for 8 sensors (0-7000)

// --- Motor Speed Settings ---
const int MAX_SPEED = 200;
const int BASE_SPEED = 150; 
const int TURN_SPEED = 120;

// --- Maze Solving Variables ---
char path[100]; 
int pathLength = 0;
int pathIndex = 0;

// --- Game Timing Variables ---
unsigned long startTime;
unsigned long dryRunDuration = 0;
unsigned long actualRunTimeLimit = 0;
const unsigned long DRY_RUN_LIMIT_MS = 3 * 60 * 1000UL; // 3 mins
const unsigned long BASE_ACTUAL_LIMIT_MS = 2 * 60 * 1000UL + 30 * 1000UL; // 2m 30s

// --- Menu States ---
enum MenuState { MAIN_MENU, CALIBRATING, DRY_RUN, ACTUAL_RUN, COMPLETED };
MenuState currentState = MAIN_MENU;
int menuOption = 0;

// --- Function Prototypes ---
void setupHardware();
void handleMenu();
void runCalibration();
void doDryRun();
void doActualRun();
void runPID(uint16_t position); // UPDATED PROTOTYPE
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
  switch (currentState) {
    case MAIN_MENU:
      handleMenu();
      break;
    case CALIBRATING:
      runCalibration();
      currentState = MAIN_MENU;
      break;
    case DRY_RUN:
      doDryRun();
      currentState = MAIN_MENU;
      break;
    case ACTUAL_RUN:
      doActualRun();
      currentState = MAIN_MENU;
      break;
    case COMPLETED:
       // Wait for user input to reset
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
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
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
}

// FIXED: Corrected voltage ranges for buttons
int readButtons() {
  int analogVal = analogRead(BUTTON_PIN);
  
  // Values based on standard resistor ladders (modify if needed)
  // Check from highest voltage down.
  
  if (analogVal > 950) return 3; // Select (Hard 5V press)
  if (analogVal > 700) return 3; // Select (Standard)
  if (analogVal > 500) return 2; // Down
  if (analogVal > 300) return 1; // Up
  
  return 0; // No button pressed (usually 0V or close to it)
}

void handleMenu() {
  int button = readButtons();
  delay(150); // Simple debounce

  if (button == 1) menuOption = (menuOption - 1 + 3) % 3; // Up
  if (button == 2) menuOption = (menuOption + 1) % 3;     // Down

  String opt1 = (menuOption == 0) ? "> 1. Calibrate" : "  1. Calibrate";
  String opt2 = (menuOption == 1) ? "> 2. Dry Run" : "  2. Dry Run";
  String opt3 = (menuOption == 2) ? "> 3. Actual Run" : "  3. Actual Run";
  displayUpdate("MAIN MENU", opt1, opt2, opt3);

  if (button == 3) { // Select
    delay(500); // Wait for release
    if (menuOption == 0) currentState = CALIBRATING;
    if (menuOption == 1) currentState = DRY_RUN;
    if (menuOption == 2) currentState = ACTUAL_RUN;
  }
}

void displayUpdate(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  display.setCursor(0, 0); display.println(line1);
  display.setCursor(0, 16); display.println(line2);
  display.setCursor(0, 32); display.println(line3);
  display.setCursor(0, 48); display.println(line4);
  display.display();
}

//void displayUpdate(String line1, String line2, String line3) {
 //   displayUpdate(line1, line2, line3, "");
//}

// =================================================================================
// CORE LOGIC: CALIBRATION & RUN MODES
// =================================================================================

void runCalibration() {
  displayUpdate("Calibrating...", "Spin bot over", "line & dark area.");
  // Spin to calibrate sensors over both white line and black floor
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    setMotorSpeeds(100, -100); 
  }
  stopMotors();
  displayUpdate("Calibration", "Complete!", "Press SELECT");
  while(readButtons() != 3) delay(10);
  delay(500);
}

// FIXED: Loop order corrected to prevent jitter
void doDryRun() {
  displayUpdate("DRY RUNNING...", "Path finding ON");
  startTime = millis();
  pathLength = 0;
  digitalWrite(LED_PIN, LOW);

  while (true) {
    // 1. READ SENSORS
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // 2. CHECK FOR END ZONE (All White)
    int whiteSensorCount = 0;
    for(int i=0; i<SensorCount; i++) {
        if(sensorValues[i] < 200) whiteSensorCount++;
    }

    if (whiteSensorCount >= 6) { 
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      dryRunDuration = millis() - startTime;
      
      // --- FIXED: Penalty Calculation ---
      unsigned long penalty = 0;
      if (dryRunDuration > DRY_RUN_LIMIT_MS) {
        penalty = dryRunDuration - DRY_RUN_LIMIT_MS;
      }
      
      // --- FIXED: Prevent Underflow ---
      if (penalty > BASE_ACTUAL_LIMIT_MS) {
        actualRunTimeLimit = 0; // Penalty ate up all the time!
      } else {
        actualRunTimeLimit = BASE_ACTUAL_LIMIT_MS - penalty;
      }

      displayUpdate("DRY RUN DONE!", "Time: " + String(dryRunDuration/1000) + "s", 
                    "Limit: " + String(actualRunTimeLimit/1000) + "s");
      
      simplifyPath(); // Apply the fixed logic
      delay(3000);
      currentState = COMPLETED;
      return;
    }

    // 3. CHECK FOR INTERSECTION PATTERNS
    const int THRESHOLD = 400;
    bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
    bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);
    
    // 4. DECISION: INTERSECTION OR PID?
    if (leftDetect || rightDetect) {
       handleIntersection(true); 
       // Reset PID error variables so we don't "jerk" coming out of a turn
       lastError = 0;
       integral = 0;
    } else {
       runPID(position);
    }
  }
}

// FIXED: Applied same jitter fix to Actual Run
void doActualRun() {
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
        handleIntersection(false); // false = Actual Run Mode
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

// FIXED: PID now accepts position as an argument
void runPID(uint16_t position) {
  int error = position - target_position; 

  integral = integral + error;
  // Anti-windup
  if (integral > 10000) integral = 10000;
  if (integral < -10000) integral = -10000;
  
  int derivative = error - lastError;
  lastError = error;

  int output = Kp * error + Ki * integral + Kd * derivative;
  
  int leftMotorSpeed = BASE_SPEED + output;
  int rightMotorSpeed = BASE_SPEED - output;

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void handleIntersection(bool isDryRun) {
  // Define sensor thresholds for white detection (low value = white)
  const int THRESHOLD = 400;
  bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
  bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);
  bool straightDetect = (sensorValues[3] < THRESHOLD) || (sensorValues[4] < THRESHOLD);

  // Safety Check: Are we still at a junction?
  if (!leftDetect && !rightDetect) return;

  // Move a bit forward to center on the intersection (Blind move)
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(100); 

  // Re-read sensors at center of intersection
  qtr.readLineWhite(sensorValues);
  leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
  rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);
  straightDetect = (sensorValues[3] < THRESHOLD) || (sensorValues[4] < THRESHOLD);

  // FIXED: Overflow protection for path array
  if (pathLength >= 99) return; 

  if (isDryRun) {
    // --- Left-Hand Rule Logic for Dry Run ---
    if (leftDetect) {
      path[pathLength++] = 'L';
      turnLeft();
    } else if (straightDetect) {
      path[pathLength++] = 'S';
      // Already moving straight
    } else if (rightDetect) {
      path[pathLength++] = 'R';
      turnRight();
    } else {
      path[pathLength++] = 'B';
      turnBack();
    }
  } else {
    // --- Replay Path for Actual Run ---
    if (pathIndex >= pathLength) return; // End of path
    char turnToTake = path[pathIndex++];
    
    switch(turnToTake) {
      case 'L': turnLeft(); break;
      case 'S': break; // Go straight
      case 'R': turnRight(); break;
    }
  }
}

void simplifyPath() {
  // Only simplify if we have at least 3 turns and the second-to-last is 'B'
  // We loop to ensure we catch nested simplifications (e.g., LBLBL -> SBL -> R)
  if (pathLength < 3) return;

  int i = 0;
  while (i <= pathLength - 3) {
    if (path[i + 1] == 'B') {
      char newTurn = '?';
      
      // --- STANDARD LEFT-HAND RULE REDUCTION TABLE ---
      if      (path[i] == 'L' && path[i+2] == 'L') newTurn = 'S'; // Left-Back-Left -> Straight
      else if (path[i] == 'L' && path[i+2] == 'R') newTurn = 'B'; // Left-Back-Right -> Back [FIXED]
      else if (path[i] == 'L' && path[i+2] == 'S') newTurn = 'R'; // Left-Back-Straight -> Right
      else if (path[i] == 'S' && path[i+2] == 'L') newTurn = 'R'; // Straight-Back-Left -> Right
      else if (path[i] == 'S' && path[i+2] == 'S') newTurn = 'B'; // Straight-Back-Straight -> Back
      else if (path[i] == 'R' && path[i+2] == 'L') newTurn = 'B'; // Right-Back-Left -> Back

      if (newTurn != '?') {
        path[i] = newTurn;
        // Shift the rest of the array to overwrite the two simplified nodes
        // We start from i+1 because path[i] is already updated
        for (int j = i + 1; j < pathLength - 2; j++) {
          path[j] = path[j + 2];
        }
        pathLength -= 2; // Path is now 2 steps shorter
        
        // Backtrack 'i' to check if this new turn created a new simplify opportunity
        // e.g. L B L -> S. Now check the node *before* this S.
        if (i > 0) i -= 2; 
        else i = -1; // Reset to start
      }
    }
    i++;
  }
}

// =================================================================================
// MOTOR CONTROL FUNCTIONS
// =================================================================================

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(leftSpeed));
  }

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

// --- SENSOR-BASED TURNING FUNCTIONS ---

void turnLeft() {
  // 1. Start spinning
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED); 
  
  // 2. Blind Spin: Rotate blindly for a fraction of a second 
  // to ensure sensors move OFF the current intersection/line.
  delay(200); 

  // 3. Sensor Hunt: Keep spinning until center sensors find the line
  while(true) {
    qtr.readLineWhite(sensorValues);
    // If center sensors (3 or 4) see the white line (value < 500)
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break;
    }
  }
  
  // 4. Stop and stabilize
  stopMotors();
  delay(50); // Optional: Brief pause to settle before PID takes over
}

void turnRight() {
  // 1. Start spinning
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  
  // 2. Blind Spin
  delay(200);

  // 3. Sensor Hunt
  while(true) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break;
    }
  }
  
  // 4. Stop
  stopMotors();
  delay(50);
}

void turnBack() {
  // 1. Start spinning (usually Right for U-turn)
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  
  // 2. Blind Spin: Longer duration to ensure we rotate past 90 degrees
  // before we start looking for the line again.
  delay(400);

  // 3. Sensor Hunt
  while(true) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
      break;
    }
  }
  
  // 4. Stop
  stopMotors();
  delay(50);
}
