#include <QTRSensors.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =================================================================================
// COMPETITIVE LINE FOLLOWER & MAZE SOLVER
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
// Sensors connected to D2 through D9
const uint8_t sensorPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

// --- Motor Driver (TB6612FNG) Pin Definitions ---
// STBY pin on driver must be connected to 5V explicitly in hardware!
#define PWMA 10  // Left Motor Speed
#define AIN1 A0  // Left Motor Direction 1
#define AIN2 A1  // Left Motor Direction 2
#define PWMB 11  // Right Motor Speed
#define BIN1 A2  // Right Motor Direction 1
#define BIN2 A3  // Right Motor Direction 2

// --- Other Hardware Pins ---
#define LED_PIN 12      // White LED to indicate "End Box" detection

// --- BUTTON PINS (Discrete Analog Inputs with External Pull-Downs) ---
// A6 and A7 are Analog ONLY. 
// Hardware setup: Button connects 5V to Pin. 10k Resistor connects Pin to GND.
#define BTN_NEXT_PIN   A7  // Button to scroll menu options
#define BTN_SELECT_PIN A6  // Button to confirm selection

// --- PID Control Constants (TUNE THESE ON TRACK!) ---
// NOTE: Start with Kp, then Kd. Keep Ki 0 unless necessary.
float Kp = 0.07;  // Proportional (Start small, e.g., 0.05 - 0.1)
float Ki = 0.00;  // Integral     (Usually 0 for line followers)
float Kd = 0.6;   // Derivative   (Damping factor, usually 5x to 10x of Kp)

// --- PID Variables ---
int lastError = 0;
int integral = 0;
int target_position = 3500; // Center for 8 sensors (0-7000 range)

// --- Motor Speed Settings ---
const int MAX_SPEED = 200;  // Max PWM (0-255)
const int BASE_SPEED = 150; // Cruising speed
const int TURN_SPEED = 130; // Speed during intersection turns

// --- Maze Solving Variables ---
char path[100];
int pathLength = 0;
int pathIndex = 0;

// --- Game Timing Variables ---
unsigned long startTime;
unsigned long dryRunDuration = 0;
unsigned long actualRunTimeLimit = 0;
const unsigned long DRY_RUN_LIMIT_MS = 3 * 60 * 1000UL; // 3 minutes standard
const unsigned long BASE_ACTUAL_LIMIT_MS = 2 * 60 * 1000UL + 30 * 1000UL; // 2m 30s

// --- Menu States ---
enum MenuState { MAIN_MENU, CALIBRATING, DRY_RUN, ACTUAL_RUN, COMPLETED };
MenuState currentState = MAIN_MENU;
int menuOption = 0; 

// --- Function Prototypes ---
void setupHardware();
void handleMenu();
void runCalibration();
bool loadCalibration();
void saveCalibration();
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
  stopMotors(); // Safety first
  
  // Try to load existing calibration from EEPROM on startup
  if (loadCalibration()) {
    displayUpdate("Bot Ready!", "Stored Calib Loaded", "Select Mode");
  } else {
    displayUpdate("Bot Ready!", "No Calib Found", "Please Calibrate");
  }
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
       // Wait for user interaction to reset to main menu
       if(readButtons() != 0) {
         delay(500);
         currentState = MAIN_MENU;
       }
       break;
  }
}

// =================================================================================
// STORAGE & CALIBRATION FUNCTIONS
// =================================================================================

void saveCalibration() {
  int address = 0;
  // Store Minimums and Maximums
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.put(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(uint16_t);
    EEPROM.put(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(uint16_t);
  }
  // Write signature byte to indicate valid data
  EEPROM.write(address, 123);
}

bool loadCalibration() {
  // We call calibrate() once to force the library to allocate the 
  // memory arrays for minimum and maximum values.
  qtr.calibrate(); 

  int address = 0;
  int signatureAddress = SensorCount * 2 * sizeof(uint16_t);
  
  // Check signature
  if (EEPROM.read(signatureAddress) != 123) {
    return false;
  }

  // Load values
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.get(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(uint16_t);
    EEPROM.get(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(uint16_t);
  }
  return true;
}

void runCalibration() {
  displayUpdate("Calibrating...", "Spin bot over", "line & dark area.");
  
  // Rotate bot back and forth
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    // Spin in place (Left motor forward, Right motor backward)
    // Reduce speed if bot spins too violently
    setMotorSpeeds(120, -120); 
  }
  stopMotors();
  
  saveCalibration();
  displayUpdate("Calibration", "Saved to Memory!", "Press SELECT");
  while(readButtons() != 2) delay(10);
  delay(500);
}

// =================================================================================
// HARDWARE & MENU FUNCTIONS
// =================================================================================

void setupHardware() {
  Serial.begin(9600);
  
  // Initialize OLED Display (Address 0x3C)
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
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

// Returns: 0 (None), 1 (Next), 2 (Select)
int readButtons() {
  // Analog Read threshold for 5V signal (active high with pull-down)
  // Pressed = ~1023, Unpressed = 0
  if (analogRead(BTN_SELECT_PIN) > 500) {
    return 2; // Select Button
  }
  if (analogRead(BTN_NEXT_PIN) > 500) {
    return 1; // Next Button
  }
  return 0;
}

void handleMenu() {
  int button = readButtons();
  delay(200); // Simple debounce

  if (button == 1) { 
    menuOption = (menuOption + 1) % 3;
  }

  String opt1 = (menuOption == 0) ? "> 1. Calibrate" : "  1. Calibrate";
  String opt2 = (menuOption == 1) ? "> 2. Dry Run" : "  2. Dry Run";
  String opt3 = (menuOption == 2) ? "> 3. Actual Run" : "  3. Actual Run";
  displayUpdate("MAIN MENU", opt1, opt2, opt3);

  if (button == 2) { 
    delay(500);
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

// =================================================================================
// CORE LOGIC: RUN MODES
// =================================================================================

void doDryRun() {
  displayUpdate("DRY RUNNING...", "Path finding ON");
  startTime = millis();
  pathLength = 0;
  digitalWrite(LED_PIN, LOW);
  
  // Reset PID history
  lastError = 0;
  integral = 0;

  while (true) {
    // IMPORTANT: Use readLineWhite for White Line on Black Surface.
    // Use readLineBlack for Black Line on White Surface.
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // Check End Zone (All sensors reading white/low value)
    int whiteSensorCount = 0;
    for(int i=0; i<SensorCount; i++) {
        if(sensorValues[i] < 200) whiteSensorCount++;
    }

    if (whiteSensorCount >= 6) { 
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      dryRunDuration = millis() - startTime;
      
      // Calculate penalty for actual run
      unsigned long penalty = 0;
      if (dryRunDuration > DRY_RUN_LIMIT_MS) {
        penalty = dryRunDuration - DRY_RUN_LIMIT_MS;
      }
      
      if (penalty > BASE_ACTUAL_LIMIT_MS) {
        actualRunTimeLimit = 0;
      } else {
        actualRunTimeLimit = BASE_ACTUAL_LIMIT_MS - penalty;
      }

      displayUpdate("DRY RUN DONE!", "Time: " + String(dryRunDuration/1000) + "s", 
                    "Limit: " + String(actualRunTimeLimit/1000) + "s");
      simplifyPath(); 
      delay(3000);
      currentState = COMPLETED;
      return;
    }

    // Check Intersections
    const int THRESHOLD = 400; // Value below which line is detected
    // Sensors 0,1 are far left. Sensors 6,7 are far right.
    bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
    bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);

    if (leftDetect || rightDetect) {
       handleIntersection(true);
       // Reset PID terms after a sharp turn/stop
       lastError = 0;
       integral = 0;
    } else {
       runPID(position);
    }
  }
}

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
  
  lastError = 0;
  integral = 0;

  while (true) {
    uint16_t position = qtr.readLineWhite(sensorValues);

    // Check Time Limit
    if (millis() - startTime > actualRunTimeLimit && actualRunTimeLimit > 0) {
      stopMotors();
      displayUpdate("GAME OVER", "Time Limit", "Exceeded!");
      delay(3000);
      currentState = COMPLETED;
      return;
    }
    
    // Check End Zone
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

    // Check Intersections
    const int THRESHOLD = 400;
    bool leftDetect = (sensorValues[0] < THRESHOLD) && (sensorValues[1] < THRESHOLD);
    bool rightDetect = (sensorValues[6] < THRESHOLD) && (sensorValues[7] < THRESHOLD);

    if (leftDetect || rightDetect) {
        handleIntersection(false);
        lastError = 0;
        integral = 0;
    } else {
        runPID(position);
    }
  }
}

// =================================================================================
// CONTROL LOGIC
// =================================================================================

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
  const int THRESHOLD = 400;
  // Inch forward to center wheels on the intersection
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(150); // Small delay to center bot

  qtr.readLineWhite(sensorValues);
  
  // Re-read sensors to determine intersection type
  bool leftDetect = (sensorValues[0] < THRESHOLD) || (sensorValues[1] < THRESHOLD);
  bool rightDetect = (sensorValues[6] < THRESHOLD) || (sensorValues[7] < THRESHOLD);
  // Center sensors
  bool straightDetect = (sensorValues[3] < THRESHOLD) || (sensorValues[4] < THRESHOLD);

  if (pathLength >= 99) return; // Prevent overflow

  if (isDryRun) {
    // Left Hand Rule Logic
    if (leftDetect) {
      path[pathLength++] = 'L';
      turnLeft();
    } else if (straightDetect) {
      path[pathLength++] = 'S';
      // Just continue straight, no turn needed
    } else if (rightDetect) {
      path[pathLength++] = 'R';
      turnRight();
    } else {
      path[pathLength++] = 'B';
      turnBack();
    }
  } else {
    // Replay Mode
    if (pathIndex >= pathLength) return;
    char turnToTake = path[pathIndex++];
    
    switch(turnToTake) {
      case 'L': turnLeft(); break;
      case 'S': break; // Drive straight through
      case 'R': turnRight(); break;
      case 'B': turnBack(); break; // Should not happen in optimized path
    }
  }
}

void simplifyPath() {
  // LBL -> S
  // LBR -> B
  // LBS -> R
  // SBL -> R
  // SBS -> B
  // RBL -> B
  
  if (pathLength < 3) return;
  
  int i = 0;
  while (i <= pathLength - 3) {
    if (path[i + 1] == 'B') {
      char newTurn = '?';
      if      (path[i] == 'L' && path[i+2] == 'L') newTurn = 'S';
      else if (path[i] == 'L' && path[i+2] == 'R') newTurn = 'B';
      else if (path[i] == 'L' && path[i+2] == 'S') newTurn = 'R';
      else if (path[i] == 'S' && path[i+2] == 'L') newTurn = 'R';
      else if (path[i] == 'S' && path[i+2] == 'S') newTurn = 'B';
      else if (path[i] == 'R' && path[i+2] == 'L') newTurn = 'B';
      
      if (newTurn != '?') {
        path[i] = newTurn;
        // Shift array to remove the two redundant turns
        for (int j = i + 1; j < pathLength - 2; j++) {
          path[j] = path[j + 2];
        }
        pathLength -= 2;
        // Backtrack to check if the new turn creates another simplification opportunity
        if (i > 0) i -= 2; 
        else i = -1; // Reset to start
      }
    }
    i++;
  }
}

// =================================================================================
// MOTOR & TURN FUNCTIONS
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

// Non-blocking turns are hard, so we use a Failsafe counter to prevent hanging
void turnLeft() {
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(300); // Wait for sensor to leave current line
  
  unsigned long turnStart = millis();
  while(millis() - turnStart < 2000) { // 2 sec timeout
    qtr.readLineWhite(sensorValues);
    // Stop when center sensor sees line
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
       stopMotors();
       delay(100);
       return;
    }
  }
  stopMotors(); // Failsafe stop
}

void turnRight() {
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(300);
  
  unsigned long turnStart = millis();
  while(millis() - turnStart < 2000) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
       stopMotors();
       delay(100);
       return;
    }
  }
  stopMotors();
}

void turnBack() {
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(600); // Needs longer time to do 180
  
  unsigned long turnStart = millis();
  while(millis() - turnStart < 3000) {
    qtr.readLineWhite(sensorValues);
    if (sensorValues[3] < 500 || sensorValues[4] < 500) {
       stopMotors();
       delay(100);
       return;
    }
  }
  stopMotors();
}