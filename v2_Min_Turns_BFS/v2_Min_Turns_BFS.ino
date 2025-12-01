#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// ================== GLOBAL CONSTANTS ==================
#define ROWS 11
#define COLS 9
#define MAX_PATH_LENGTH 400
#define NODE_DISTANCE 100
#define PPR 11
#define WHEEL_DIAMETER 6.50
#define PI 3.14159265358979
#define GEAR_RATIO 30.0
#define BASE_WIDTH 203.2
const float GRID_STEP_DISTANCE = 10.0;
const int ACTION_DELAY = 1000;
const int IR_TURN_SENSOR_PIN = 11; 

// ================== GLOBAL VARIABLES ==================
int currentAngle = 145;
float forwardDistance = 0;
bool hasPerformedAction = false;
unsigned long actionStartTime = 0;
int delays = 400;
int currentMainGoalIndex = 0;
int totalMainGoals = 3;
bool firstBlueOrGreenDetected = false;
bool resetPathFlag = false;
bool goalsDetected = false;

// ================== STRUCTURES & ENUMS ==================
enum Color { RED, GREEN, BLUE, NONE, NO_COLOR };
Color detectedColor = NO_COLOR;

enum Direction : byte { NORTH, EAST, SOUTH, WEST };
enum Action { FORWARD, HARD_LEFT, HARD_RIGHT, U_TURN, INVALID };
enum RobotState { 
    MOVING_TO_MAIN_GOAL, 
    APPROACHING_MAIN_GOAL, 
    DETECTING_COLOR, 
    MOVING_TO_SPECIAL_GOAL, 
    WAITING_AT_SPECIAL_GOAL, 
    MOVING_TO_PARKING 
};

RobotState robotState = MOVING_TO_MAIN_GOAL;
unsigned long goalReachedTime = 0;

struct Point {
    byte x;
    byte y;
};

struct BFSPoint {
    byte x;
    byte y;
    int depth;
    int turns;
    Direction dir;
};

// ================== SENSOR & MOTOR OBJECTS ==================
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Servo myservo;
QTRSensors qtr;

// ================== SENSOR CONFIGURATION ==================
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
unsigned int normalizedValues[SensorCount];
const unsigned int calibratedMin[SensorCount] = {42, 42, 41, 43, 41, 43, 42, 44};
const unsigned int calibratedMax[SensorCount] = {901, 898, 904, 903, 901, 911, 898, 920};
const int MIN_BRIGHTNESS = 500;

// ================== MOTOR PIN CONFIGURATION ==================
const int Motor1_forward = 51;
const int Motor1_backward = 50;
const int Motor2_forward = 48;
const int Motor2_backward = 49;
const int ENA = 3;
const int ENB = 4;

// ================== PID PARAMETERS ==================
const double KP = 0.08;
const double KI = 0.0;
const double KD = 1.7;
const unsigned char motor_speed = 245;
const unsigned char motor_backspeed = 110;
double last_error = 0;
double Goal = 3500;
double integral = 0;
float KpTurn = 0.08;
float Kdturn = 0.1;
int errorTurn = 0, last_errorTurn = 0, derivativeYurn = 0;

// ================== ENCODER CONFIGURATION ==================
const int ENCA_R = 2;
const int ENCB_R = 9;
const int ENCA_L = 18;
const int ENCB_L = 8;
volatile long right_pulse_count = 0;
volatile long left_pulse_count = 0;

// ================== GRID & PATH CONFIGURATION ==================
int grid[ROWS][COLS] = {
    {1,1,1,1,1,1,1,1,1},
    {1,0,1,0,1,0,1,1,1},
    {1,0,1,0,1,0,1,1,1},
    {1,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,1,1,1},
    {1,0,0,0,0,0,1,1,1},
    {1,0,0,0,0,0,1,1,1},
    {1,0,1,0,0,0,1,1,1},
    {1,1,1,1,1,0,1,1,1},
    {1,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,1,1,1}
};

Point mainGoals[3];
Point currentPos = {5, 9};
Direction currentDir = NORTH;
Point parking = {7,3};
Point specialGoal;

Point pathQueue[MAX_PATH_LENGTH];
int queueFront = 0;
int queueRear = -1;
int itemCount = 0;

BFSPoint bfsQueue[MAX_PATH_LENGTH];
int bfsFront = 0;
int bfsRear = 0;
int bfsDist[ROWS][COLS];
int bfsMinTurns[ROWS][COLS];
Direction bfsArrivalDir[ROWS][COLS];
Point parent[ROWS][COLS];
bool visitedGoals[3] = {false, false, false};

// ================== IR SENSOR CONFIGURATION ==================
const int back_Ir[5] = {46, 45, 44, 43, 42}; 
bool Ir_values[9]; 
const int irSensorPins[9] = {40, 37, 34, 38, 35, 32, 39, 36, 33};
Point irSensorMap[9] = {
    {4, 6},  {4,5},   {4,4},
    {2,6},  {2, 5},   {2, 4},
    {3,6},  {3, 5},   {3, 4}
};

// ================== FUNCTION DECLARATIONS ==================
void pushToQueue(Point p);
Point popFromQueue();
void updateGridForPathPlanning(Point target);
void bfsInitialize(Point start, Point end, Direction startDir);
void reconstructPath(Point start, Point end);
Direction calculateDirection(Point next);
Action determineTurn(Direction required);
void executeAction(Action action);
bool followPathToGoal();
void setupQTR();
void readAndNormalizeSensors();
void setupMotors();
void controlMotors(double leftSpeed, double rightSpeed);
void pid_linefollowing();
void pid_linefollowing1();
void back_linefollowing();
void read_grid();
void TurnRight1(int targetTicks);
void TurnLeft(int targetTicks);
void U_turn(int targetTicks);
bool detectJunction();
void rightEncoder();
void leftEncoder();
void detectColor();
void detectGoalsFromIRSensors();
void logState(Action action);
void drop();
void drop1();
void lift();
void moveForward(float distance_cm);
void moveForward1(float distance_cm);
void moveBackward(float distance_cm);
void executeInitialPath();

// ================== QUEUE OPERATIONS ==================
void pushToQueue(Point p) {
    if (itemCount < MAX_PATH_LENGTH) {
        queueRear = (queueRear + 1) % MAX_PATH_LENGTH;
        pathQueue[queueRear] = p;
        itemCount++;
    }
}

Point popFromQueue() {
    Point p = pathQueue[queueFront];
    queueFront = (queueFront + 1) % MAX_PATH_LENGTH;
    itemCount--;
    return p;
}

// ================== PATH PLANNING FUNCTIONS ==================
void updateGridForPathPlanning(Point target) {
    int tempGrid[ROWS][COLS];
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            tempGrid[y][x] = grid[y][x];
        }
    }

    for (int i = 0; i < 3; i++) {
        Point p = mainGoals[i];
        if (tempGrid[p.y][p.x] != 1) {
            tempGrid[p.y][p.x] = 0;
        }
    }

    for (int i = 0; i < totalMainGoals; i++) {
        if (!visitedGoals[i]) {
            Point goal = mainGoals[i];
            if (goal.x != target.x || goal.y != target.y) {
                tempGrid[goal.y][goal.x] = 1;
            }
        }
    }

    tempGrid[target.y][target.x] = 0;

    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            grid[y][x] = tempGrid[y][x];
        }
    }
}

void bfsInitialize(Point start, Point end, Direction startDir) {
    // Initialize tracking arrays
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            bfsDist[y][x] = -1;
            bfsMinTurns[y][x] = 1000000; // Large initial value
            parent[y][x] = {(byte)-1, (byte)-1};
        }
    }

    // Reset queue
    bfsFront = 0;
    bfsRear = 0;

    // Initialize start node
    bfsDist[start.y][start.x] = 0;
    bfsMinTurns[start.y][start.x] = 0;
    bfsArrivalDir[start.y][start.x] = startDir;
    parent[start.y][start.x] = {(byte)-1, (byte)-1};
    bfsQueue[bfsRear++] = {start.x, start.y, 0, 0, startDir};

    // Direction vectors (N, E, S, W)
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {-1, 0, 1, 0};

    while (bfsFront < bfsRear) {
        BFSPoint u = bfsQueue[bfsFront++];

        // Skip if non-optimal path
        if (u.turns > bfsMinTurns[u.y][u.x]) {
            continue;
        }

        // Explore neighbors
        for (int i = 0; i < 4; i++) {
            byte nx = u.x + dx[i];
            byte ny = u.y + dy[i];
            Direction movementDir = static_cast<Direction>(i);

            // Boundary and obstacle check
            if (nx >= COLS || ny >= ROWS || grid[ny][nx] != 0) {
                continue;
            }

            int newDepth = u.depth + 1;
            int newTurns = u.turns;
            
            // Add turn cost if direction changed
            if (movementDir != u.dir) {
                newTurns++;
            }

            // Update if unvisited or better path found
            if (bfsDist[ny][nx] == -1 || 
                (bfsDist[ny][nx] == newDepth && newTurns < bfsMinTurns[ny][nx])) {
                
                bfsDist[ny][nx] = newDepth;
                bfsMinTurns[ny][nx] = newTurns;
                bfsArrivalDir[ny][nx] = movementDir;
                parent[ny][nx] = {u.x, u.y};
                
                // Add to queue if space available
                if (bfsRear < MAX_PATH_LENGTH) {
                    bfsQueue[bfsRear++] = {nx, ny, newDepth, newTurns, movementDir};
                }
            }
        }
    }
}

void reconstructPath(Point start, Point end) {
    itemCount = 0;
    queueFront = 0;
    queueRear = -1;

    if (start.x == end.x && start.y == end.y) {
        return;
    }

    if (bfsDist[end.y][end.x] == -1) {
        Serial.println("No path to goal");
        return;
    }

    Point tempPath[MAX_PATH_LENGTH];
    int pathIndex = 0;
    Point current = end;

    // Backtrack from end to start
    while (current.x != start.x || current.y != start.y) {
        tempPath[pathIndex++] = current;
        Point prev = parent[current.y][current.x];
        if (prev.x == 255 && prev.y == 255) { // -1 becomes 255 in byte
            Serial.println("Path reconstruction failed");
            return;
        }
        current = prev;
    }
    tempPath[pathIndex++] = start;

    // Reverse path and add to queue
    for (int i = pathIndex - 2; i >= 0; i--) {
        pushToQueue(tempPath[i]);
    }
}

// ================== NAVIGATION FUNCTIONS ==================
Direction calculateDirection(Point next) {
    int dx = next.x - currentPos.x;
    int dy = next.y - currentPos.y;

    if (dx == 1) return EAST;
    if (dx == -1) return WEST;
    if (dy == 1) return SOUTH;
    if (dy == -1) return NORTH;

    return currentDir;
}

Action determineTurn(Direction required) {
    int currentDirValue = static_cast<int>(currentDir);
    int requiredDirValue = static_cast<int>(required);
    int diff = (requiredDirValue - currentDirValue + 4) % 4;

    switch (diff) {
        case 0: return FORWARD;
        case 1: return HARD_RIGHT;
        case 2: return U_TURN;
        case 3: return HARD_LEFT;
        default: return INVALID;
    }
}

void executeAction(Action action) {
    switch (action) {
        case FORWARD:
            moveForward(3);
            break;
        case HARD_LEFT:
            TurnLeft(195);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 3) % 4);
            break;
        case HARD_RIGHT:
            TurnRight1(235);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 1) % 4);
            break;
        case U_TURN:
            U_turn(405);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 2) % 4);
            break;
        default: break;
    }
}
bool followPathToGoal() {
    static bool firstPathExecution = true;
    bool fakeJunctionTriggered = false;

    if (resetPathFlag) {
        firstPathExecution = true;
        resetPathFlag = false;
    }

    if (firstPathExecution && itemCount > 0 && robotState != WAITING_AT_SPECIAL_GOAL) {
        fakeJunctionTriggered = true;
        firstPathExecution = false;
        Serial.println("Fake junction triggered for new path");
    }

    if (itemCount > 0) {
        if (fakeJunctionTriggered || detectJunction()) {
            Point next = popFromQueue();
            Direction requiredDir = calculateDirection(next);
            Action action = determineTurn(requiredDir);
            if (action != INVALID) {
                Serial.print("Executing action for node: (");
                Serial.print(next.x);
                Serial.print(",");
                Serial.print(next.y);
                Serial.println(")");
                executeAction(action);
                currentPos = next;
                logState(action);
            }
        } else {
            pid_linefollowing();
        }
        return false;
    } else {
        controlMotors(0, 0);
        return true;
    }
}

// ================== SENSOR FUNCTIONS ==================
void setupQTR() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10,A9,A8}, SensorCount);
    qtr.setEmitterPin(2);
    delay(500);
    Serial.println("Using hardcoded calibration values.");
}

void readAndNormalizeSensors() {
    qtr.read(sensorValues);
    for (int i = 0; i < SensorCount; i++) {
        if (sensorValues[i] < calibratedMin[i]) {
            normalizedValues[i] = 0;
        } else if (sensorValues[i] > calibratedMax[i]) {
            normalizedValues[i] = 1000;
        } else {
            normalizedValues[i] = ((sensorValues[i] - calibratedMin[i]) * 1000) / (calibratedMax[i] - calibratedMin[i]);
        }
    }
}

// ================== MOTOR CONTROL FUNCTIONS ==================
void setupMotors() {
    pinMode(Motor1_forward, OUTPUT);
    pinMode(Motor1_backward, OUTPUT);
    pinMode(Motor2_forward, OUTPUT);
    pinMode(Motor2_backward, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void controlMotors(double leftSpeed, double rightSpeed) {
    digitalWrite(Motor1_forward, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(Motor1_backward, leftSpeed < 0 ? HIGH : LOW);
    digitalWrite(Motor2_forward, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(Motor2_backward, rightSpeed < 0 ? HIGH : LOW);
    analogWrite(ENA, abs(leftSpeed));
    analogWrite(ENB, abs(rightSpeed));
}

void pid_linefollowing() {
    readAndNormalizeSensors();
    uint16_t position = qtr.readLineBlack(normalizedValues);
    int error = Goal - position;
    integral += error;
    double adjustment = KP * error + KD * (error - last_error);
    last_error = error;

    double rightMotorSpeed = motor_speed - adjustment;
    double leftMotorSpeed = motor_speed + adjustment;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

    controlMotors(leftMotorSpeed, rightMotorSpeed-40);
}

void pid_linefollowing1() {
    readAndNormalizeSensors();
    uint16_t position = qtr.readLineBlack(normalizedValues);
    int error = Goal - position;
    integral += error;
    double adjustment = KP * error + KD * (error - last_error);
    last_error = error;

    double rightMotorSpeed = motor_speed - adjustment;
    double leftMotorSpeed = motor_speed + adjustment;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, 150);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 150);

    controlMotors(leftMotorSpeed, rightMotorSpeed-20);
}

void back_linefollowing() {
    bool leftSensor = !digitalRead(back_Ir[1]);
    bool rightSensor = !digitalRead(back_Ir[2]);

    if (leftSensor && rightSensor) {
        controlMotors(-150, -165);
    }
    else if (leftSensor) {
        controlMotors(-150,-80);
    }
    else if (rightSensor) {
        controlMotors(-80, -150);
    }
    else {
        controlMotors(-150, -165);
    }
}

// ================== IR SENSOR FUNCTIONS ==================
void read_grid() {
    for (uint8_t i = 0; i < 9; i++) {
        Ir_values[i] = digitalRead(irSensorPins[i]);
    }
    int sum = 0;
    for (uint8_t i = 0; i < 9; i++) {
        sum += Ir_values[i];
    }
    for (uint8_t i = 0; i < 9; i++) {
        Ir_values[i] = !Ir_values[i];
    }

    detectGoalsFromIRSensors();

    if (sum != 3) {
        moveForward(4);
        controlMotors(-100,-100);
        delay(50);
        moveBackward(6);
        controlMotors(100,100);
        delay(50);
        controlMotors(0,0);
        delay(500);
        read_grid();
    }
}

// ================== MOVEMENT FUNCTIONS ==================
void TurnRight1(int targetTicks) {
    moveForward(5);
    controlMotors(-255, -255);
    delay(130);
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) {
        controlMotors(255, -255);
    }
    controlMotors(-255, 255);
    delay(100);
    controlMotors(0, 0);
    delay(10);
}

void TurnLeft(int targetTicks) {
    moveForward(5);
    controlMotors(-255, -255);
    delay(130);
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) {
        controlMotors(-180, 150);
    }
    controlMotors(180, -180);
    delay(80);
    controlMotors(0, 0);
    delay(10);
}

void TurnLeft1(int targetTicks) {
    moveForward(7);
    controlMotors(-255, -255);
    delay(130);
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) {
        controlMotors(-180, 150);
    }
    controlMotors(180, -180);
    delay(80);
    controlMotors(0, 0);
    delay(10);
}

void U_turn(int targetTicks) {
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) {
        controlMotors(-200, 200);
    }
    controlMotors(200, -200);
    delay(100);
    controlMotors(0, 0);
    delay(40);
}

// ================== JUNCTION DETECTION ==================
bool detectJunction() {
    static bool previousJunctionState = false;
    bool currentJunctionState = false;

    readAndNormalizeSensors();
    currentJunctionState = true;
    for (int i = 0; i < 8; i++) {
        if (normalizedValues[i] < 700) {
            currentJunctionState = false;
            break;
        }
    }

    if (currentJunctionState && !previousJunctionState) {
        Serial.println("Junction detected");
        previousJunctionState = currentJunctionState;
        return true;
    }

    previousJunctionState = currentJunctionState;
    return false;
}

// ================== ENCODER FUNCTIONS ==================
void rightEncoder() { right_pulse_count++; }
void leftEncoder() { left_pulse_count++; }

// ================== COLOR DETECTION ==================
void detectColor() {
    uint16_t r, g, b, c;
    bool validColorDetected = false;
    
    while (!validColorDetected) {
        tcs.getRawData(&r, &g, &b, &c);

        if (c < MIN_BRIGHTNESS) {
            Serial.println("Low light condition");
            delay(500);
            continue;
        }

        float redRatio = r;
        float greenRatio = g;
        float blueRatio = b;

        if (redRatio > greenRatio && redRatio > blueRatio) {
            detectedColor = RED;
            specialGoal = {1, 7};
            Serial.println("Detected Red");
            
         
            validColorDetected = true;
        } 
        else if (greenRatio > redRatio && greenRatio > blueRatio) {
            detectedColor = GREEN;
            if (!firstBlueOrGreenDetected) {
                specialGoal = {1, 1};
                firstBlueOrGreenDetected = true;
            } else {
                specialGoal = {5, 1};
            }
            Serial.println("Detected Green");
          
            validColorDetected = true;
        } 
        else if (blueRatio > greenRatio && blueRatio > redRatio) {
            detectedColor = BLUE;
            if (!firstBlueOrGreenDetected) {
                specialGoal = {1, 1};
                firstBlueOrGreenDetected = true;
            } else {
                specialGoal = {5, 1};
            }
            Serial.println("Detected Blue");
           
            validColorDetected = true;
        } 
        else {
            Serial.println("No clear color detected");
            delay(500);
        }
    }
   
}

// ================== GOAL DETECTION ==================
void detectGoalsFromIRSensors() {
    int detectedCount = 0;
    for (int i = 0; i < 9; i++) {
        int sensorValue = digitalRead(irSensorPins[i]);
        if (sensorValue == HIGH && detectedCount < 3) {
            mainGoals[detectedCount++] = irSensorMap[i];
            Serial.print("Goal detected at: (");
            Serial.print(irSensorMap[i].x);
            Serial.print(", ");
            Serial.print(irSensorMap[i].y);
            Serial.println(")");
        }
    }
    
    if (detectedCount > 0) {
        goalsDetected = true;
        totalMainGoals = detectedCount;
        for (int i = 0; i < 3; i++) {
            visitedGoals[i] = false;
        }
    }
}

// ================== UTILITY FUNCTIONS ==================
void logState(Action action) {
    Serial.print("Position: (");
    Serial.print(currentPos.x);
    Serial.print(",");
    Serial.print(currentPos.y);
    Serial.print(") Direction: ");
    switch (currentDir) {
        case NORTH: Serial.print("NORTH"); break;
        case EAST:  Serial.print("EAST");  break;
        case SOUTH: Serial.print("SOUTH"); break;
        case WEST:  Serial.print("WEST");  break;
    }
    Serial.print(" Action: ");
    switch (action) {
        case U_TURN:     Serial.println("U_TURN");    break;
        case FORWARD:    Serial.println("FORWARD");   break;
        case HARD_LEFT:  Serial.println("HARD_LEFT"); break;
        case HARD_RIGHT: Serial.println("HARD_RIGHT");break;
        case INVALID:    Serial.println("INVALID");   break;
    }
}

void drop() {
    for (int angle = 60; angle <= 140; angle++) {
        myservo.write(angle);
        delay(5);
    }
    currentAngle = 140;
}

void drop1() {
    for (int angle = 60; angle <= 140; angle++) {
        myservo.write(angle);
        delay(10);
    }
    currentAngle = 140;
}

void lift() {
    for (int angle = 140; angle >= 60; angle--) {
        myservo.write(angle);
        delay(10);
    }
    currentAngle = 60;
}

void moveForward(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) {
        pid_linefollowing();
    }
    controlMotors(0, 0);
}

void moveForward1(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0;
    right_pulse_count = 0;
    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) {
        pid_linefollowing1();
    }
    controlMotors(0, 0);
}

void moveBackward(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0;
    right_pulse_count = 0;
    controlMotors(-100,0);
    delay(30);

    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) {
        back_linefollowing();
    }
    controlMotors(0, 0);
}

// ================== INITIAL PATH EXECUTION ==================
void executeInitialPath() {
    int junctionCounter = 0;
    while (junctionCounter < 5) {
        if (detectJunction()) {
            junctionCounter++;
            Serial.print("Junction Count: ");
            Serial.println(junctionCounter);

            if (junctionCounter == 2) {
                TurnLeft1(175);
                moveBackward(18);
                read_grid();
                pid_linefollowing();
            } else if (junctionCounter == 3) {
                TurnRight1(200);
                
            } else if (junctionCounter == 5) {
                TurnLeft(190);
            }
        } else {
            pid_linefollowing();
        }
    }
    controlMotors(0,0);
    Serial.println("Initial path completed. Following optimized path now.");
}

// ================== MAIN SETUP AND LOOP ==================
void setup() {
    Serial.begin(9600);

    // Initialize components
    setupQTR();
    setupMotors();
    myservo.attach(12);
    myservo.write(currentAngle);

    // Initialize pins
    pinMode(IR_TURN_SENSOR_PIN, INPUT);
    for (int i = 0; i < 5; i++) {
        pinMode(back_Ir[i], INPUT);
    }
    for (int i = 0; i < 9; i++) {
        pinMode(irSensorPins[i], INPUT);
    }

    // Initialize encoders
    pinMode(ENCA_R, INPUT);
    pinMode(ENCA_L, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_R), rightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_L), leftEncoder, RISING);

    
    // Initialize color sensor
    if (tcs.begin()) {
        Serial.println("Color sensor ready");
    } else {
        Serial.println("Sensor not found");
        while (1);
    }
    
    // Execute initial path
    executeInitialPath();
}

void loop() {
    switch (robotState) {
        case MOVING_TO_MAIN_GOAL:
            if (currentMainGoalIndex >= totalMainGoals) {
                robotState = MOVING_TO_PARKING;
                break;
            }
            if (itemCount == 0) {
                Serial.print("Planning path to main goal ");
                Serial.print(currentMainGoalIndex);
                Serial.print(": (");
                Serial.print(mainGoals[currentMainGoalIndex].x);
                Serial.print(",");
                Serial.print(mainGoals[currentMainGoalIndex].y);
                Serial.println(")");
                updateGridForPathPlanning(mainGoals[currentMainGoalIndex]);
                bfsInitialize(currentPos, mainGoals[currentMainGoalIndex], currentDir);
                reconstructPath(currentPos, mainGoals[currentMainGoalIndex]);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                Serial.println("Main goal path complete");
                controlMotors(-255, -255);
                delay(40);
                controlMotors(0,0);
                robotState = APPROACHING_MAIN_GOAL;
            }
            break;

        case APPROACHING_MAIN_GOAL:
        {
            static bool initialized = false;
            if (!initialized) {
                initialized = true;
                Serial.println("Approaching box...");
            }
            pid_linefollowing();
            int irState = digitalRead(IR_TURN_SENSOR_PIN);
            if (irState == LOW) {
                Serial.println("Object detected!");
                controlMotors(-220, -255);
                delay(130);
                controlMotors(0, 0);
                delay(10);
               
                robotState = DETECTING_COLOR;
                initialized = false;
            }
            break;
        }
        case DETECTING_COLOR:
        {
            moveForward(4);
            lift();
            controlMotors(0,0);
            detectColor();
            visitedGoals[currentMainGoalIndex] = true;     
            updateGridForPathPlanning(specialGoal);
            bfsInitialize(currentPos, specialGoal, currentDir);
            reconstructPath(currentPos, specialGoal);
          
            if (itemCount == 0) {
                Serial.println("Warning: No path to special goal, retrying path planning");
            } else {
                resetPathFlag = true;
            }
            Serial.println("Color detected, moving to special goal");
            robotState = MOVING_TO_SPECIAL_GOAL;
            break;
        }
        case MOVING_TO_SPECIAL_GOAL:
            if (itemCount == 0) {
                Serial.print("Planning path to special goal: (");
                Serial.print(specialGoal.x);
                Serial.print(",");
                Serial.print(specialGoal.y);
                Serial.println(")");
                updateGridForPathPlanning(specialGoal);
                bfsInitialize(currentPos, specialGoal, currentDir);
                reconstructPath(currentPos, specialGoal);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                controlMotors(-220,-255);
                delay(110);
                controlMotors(0,0);
                robotState = WAITING_AT_SPECIAL_GOAL;
                goalReachedTime = millis();
                
                Serial.println("Special goal reached");
            }
            break;

case WAITING_AT_SPECIAL_GOAL:
{
    Serial.print("Executing action for color: ");
    switch (detectedColor) {
        case BLUE:
        case GREEN:
        {
            Serial.println(detectedColor == BLUE ? "BLUE" : "GREEN");
            if (specialGoal.x == 1 && specialGoal.y == 1) {
                // First blue/green block handling
                moveForward(6);
                drop();
                moveBackward(8);
                currentDir = NORTH;
                currentPos = {1, 2};
            } else {
                // Second blue/green block handling
                controlMotors(-150,-150);  // Braking
                delay(30);                 // Minimal hardware settling
                moveForward1(6);           // Slow approach
                drop();
                moveBackward(8);
                currentDir = NORTH;
                currentPos = {5, 2};
            }
            break;
        }
        case RED:
        {
            Serial.println("RED");
            drop();
            moveForward(21);      // Push block into goal
            moveBackward(21);     // Return to start position
            currentDir = SOUTH;
            currentPos = {1, 6};
            break;
        }
        default:
            Serial.println("NO_COLOR");
            break;
    }
    controlMotors(0, 0);  // Ensure motors stop
    
    // Immediate state transition
    detectedColor = NO_COLOR;
    currentMainGoalIndex++;
    resetPathFlag = false;
    robotState = (currentMainGoalIndex < totalMainGoals) 
                 ? MOVING_TO_MAIN_GOAL 
                 : MOVING_TO_PARKING;
    break;
}
        case MOVING_TO_PARKING:
            if (itemCount == 0) {
                Serial.println("Planning path to parking");
                updateGridForPathPlanning(parking);
                bfsInitialize(currentPos, parking, currentDir);
                reconstructPath(currentPos, parking);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                controlMotors(100, 100);
                delay(400);
                controlMotors(-200, -200);
                delay(100);
                controlMotors(0,0);
                delay(10000);
                Serial.println("Parking reached");
                while (1);
            }
            break;
    }
}