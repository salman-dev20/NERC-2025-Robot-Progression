#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// ================== GLOBAL CONSTANTS ==================
#define ROWS 11
#define COLS 9
#define MAX_PATH_LENGTH 100 
#define NODE_DISTANCE 100
#define PPR 11
#define WHEEL_DIAMETER 6.50
#define PI 3.14159265358979
#define GEAR_RATIO 30.0
#define BASE_WIDTH 203.2
const float GRID_STEP_DISTANCE = 10.0;
const int ACTION_DELAY = 1000;
const int IR_TURN_SENSOR_PIN = 11; 

// ===== STPO TUNING PARAMETERS (SECONDS) =====
// Adjust these based on real-world observation
const float TIME_SEGMENT = 1.0;   // Time to travel 1 grid block (e.g., 1.0 second)
const float TIME_TURN_90 = 2.5;   // Time to make a 90 degree turn (e.g., 2.5 seconds)

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
    int8_t x;
    int8_t y;
};

// ================== STPO ALGORITHM STRUCTURES ==================
const int DIRS = 4;
const float INF_TIME = 1e9;

struct State {
    byte x;
    byte y;
    Direction dir; // the *incoming* direction used to reach this cell
    float time;    // accumulated time to reach this state
};

// parent state for reconstructing path
struct ParentState {
    byte px;
    byte py;
    Direction pdir;
    bool valid;
};

// Distances and parent arrays (global; reuse memory across calls)
float distSTPO[ROWS][COLS][DIRS];
ParentState parentSTPO[ROWS][COLS][DIRS];
bool closedSTPO[ROWS][COLS][DIRS];

// A small open list (array)
State openList[MAX_PATH_LENGTH];
int openCount = 0;

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

// ================== FORWARD DECLARATIONS ==================
void controlMotors(double leftSpeed, double rightSpeed);
void pid_linefollowing();
void moveForward(float distance_cm);
void TurnLeft(int targetTicks);
void TurnRight1(int targetTicks);
void U_turn(int targetTicks);
void dijkstraSTPO(Point start, Point end, Direction startDir, float k, float seg_time);
void reconstructPathSTPO(Point start, Point end);

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

// ================== STPO IMPLEMENTATION ==================

// helper: convert Direction enum to index 0..3
inline int dirToIndex(Direction d) {
    return static_cast<int>(d) & 0x03;
}

// helper: compute turn angle between two directions (degrees)
inline int turnAngle(Direction from, Direction to) {
    int a = abs(static_cast<int>(from) - static_cast<int>(to));
    int angle = a * 90; // 0, 90, 180, 270
    if (angle > 180) angle = 360 - angle;
    return angle; // 0, 90, 180
}

// clear/open-list helpers
void openListClear() {
    openCount = 0;
}
void openListPush(State s) {
    if (openCount < MAX_PATH_LENGTH) {
        openList[openCount++] = s;
    }
}
int openListPopMinIndex() {
    if (openCount == 0) return -1;
    int best = 0;
    float bestTime = openList[0].time;
    for (int i = 1; i < openCount; ++i) {
        if (openList[i].time < bestTime) {
            bestTime = openList[i].time;
            best = i;
        }
    }
    return best;
}
State openListPopMin() {
    int idx = openListPopMinIndex();
    State s = openList[idx];
    // remove element by shifting last into idx
    openList[idx] = openList[openCount - 1];
    openCount--;
    return s;
}

// Main STPO function
void dijkstraSTPO(Point start, Point end, Direction startDir, float k, float seg_time) {
    // init arrays
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            for (int d = 0; d < DIRS; d++) {
                distSTPO[y][x][d] = INF_TIME;
                parentSTPO[y][x][d].valid = false;
                closedSTPO[y][x][d] = false;
            }
        }
    }

    // clear pathQueue
    itemCount = 0;
    queueFront = 0;
    queueRear = -1;

    // open list
    openListClear();

    // Start state
    State s0;
    s0.x = start.x;
    s0.y = start.y;
    s0.dir = startDir;
    s0.time = 0.0f;

    distSTPO[start.y][start.x][dirToIndex(startDir)] = 0.0f;
    parentSTPO[start.y][start.x][dirToIndex(startDir)].valid = false; // root
    openListPush(s0);

    // neighbor movements: N,E,S,W (note: same ordering used in your BFS)
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {-1, 0, 1, 0};

    while (openCount > 0) {
        State cur = openListPopMin();

        // if we've already processed this state as closed, skip
        if (closedSTPO[cur.y][cur.x][dirToIndex(cur.dir)]) continue;
        closedSTPO[cur.y][cur.x][dirToIndex(cur.dir)] = true;

        // Explore neighbors
        for (int i = 0; i < 4; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            
            // Bounds and Obstacle check
            if (nx < 0 || nx >= COLS || ny < 0 || ny >= ROWS) continue;
            if (grid[ny][nx] != 0) continue; // obstacle

            Direction moveDir = static_cast<Direction>(i); 
            // Travel time for the edge
            float Te = seg_time;

            // Turn penalty
            float P = 0.0f;
            if (!(cur.x == start.x && cur.y == start.y && cur.time == 0.0f && dirToIndex(cur.dir) == dirToIndex(startDir))) {
                // general case
                int angle = turnAngle(cur.dir, moveDir); // 0,90,180
                P = k * ( (float)angle / 90.0f );
            } else {
                // At the very start
                int angle = turnAngle(cur.dir, moveDir);
                P = k * ( (float)angle / 90.0f ); 
            }

            float newTime = cur.time + Te + P;
            int midx = dirToIndex(moveDir);
            
            if (newTime < distSTPO[ny][nx][midx]) {
                distSTPO[ny][nx][midx] = newTime;
                parentSTPO[ny][nx][midx].px = cur.x;
                parentSTPO[ny][nx][midx].py = cur.y;
                parentSTPO[ny][nx][midx].pdir = cur.dir;
                parentSTPO[ny][nx][midx].valid = true;

                State next;
                next.x = nx;
                next.y = ny;
                next.dir = moveDir; 
                next.time = newTime;

                openListPush(next);
            }
        } 
    } 

    reconstructPathSTPO(start, end);
}

void reconstructPathSTPO(Point start, Point end) {
    // clear queue variables
    itemCount = 0;
    queueFront = 0;
    queueRear = -1;

    // Check if end reachable
    float bestTime = INF_TIME;
    int bestDirIdx = -1;
    for (int d = 0; d < DIRS; ++d) {
        if (distSTPO[end.y][end.x][d] < bestTime) {
            bestTime = distSTPO[end.y][end.x][d];
            bestDirIdx = d;
        }
    }

    if (bestDirIdx == -1 || bestTime >= INF_TIME) {
        Serial.println("STPO: No path to goal");
        return;
    }

    Point cur = end;
    int curDirIdx = bestDirIdx;

    Point tempPath[MAX_PATH_LENGTH];
    int pathLen = 0;

    // Backtrack
    while (true) {
        tempPath[pathLen++] = cur;
        ParentState ps = parentSTPO[cur.y][cur.x][curDirIdx];
        if (!ps.valid) {
            if (cur.x == start.x && cur.y == start.y) {
                break;
            } else {
                Serial.println("STPO: Path reconstruction failed (invalid parent)");
                return;
            }
        }
        Point prev = { ps.px, ps.py };
        Direction incomingDirToPrev = ps.pdir; 
        cur = prev;
        curDirIdx = dirToIndex(incomingDirToPrev);

        if (pathLen >= MAX_PATH_LENGTH - 2) {
            Serial.println("STPO: Path too long");
            return;
        }
    }

    // Reverse and Push
    Serial.print("STPO Steps: ");
    Serial.println(pathLen - 1);
    
    for (int i = pathLen - 2; i >= 0; --i) {
        pushToQueue(tempPath[i]);
    }
}

// ================== HELPER FUNCTIONS (Path Planning) ==================

void updateGridForPathPlanning(Point target) {
    int tempGrid[ROWS][COLS];
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            tempGrid[y][x] = grid[y][x];
        }
    }
    // Clear main goals
    for (int i = 0; i < 3; i++) {
        Point p = mainGoals[i];
        if (p.x >= 0 && p.x < COLS && p.y >= 0 && p.y < ROWS) {
            if (grid[p.y][p.x] != 1) tempGrid[p.y][p.x] = 0;
        }
    }
    // Mark unvisited as obstacles
    for (int i = 0; i < totalMainGoals; i++) {
        if (!visitedGoals[i]) {
            Point goal = mainGoals[i];
            if (goal.x != target.x || goal.y != target.y) {
                tempGrid[goal.y][goal.x] = 1;
            }
        }
    }
    tempGrid[target.y][target.x] = 0;
    
    // Apply
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            grid[y][x] = tempGrid[y][x];
        }
    }
}

// ================== NAVIGATION ==================
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
        case FORWARD: moveForward(3); break;
        case HARD_LEFT: TurnLeft(195); currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 3) % 4); break;
        case HARD_RIGHT: TurnRight1(235); currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 1) % 4); break;
        case U_TURN: U_turn(405); currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 2) % 4); break;
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
    }

    if (itemCount > 0) {
        if (fakeJunctionTriggered || detectJunction()) {
            Point next = popFromQueue();
            Direction requiredDir = calculateDirection(next);
            Action action = determineTurn(requiredDir);
            
            if (action != INVALID) {
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

// ================== HARDWARE HELPERS (Standard) ==================
void setupQTR() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10,A9,A8}, SensorCount);
    qtr.setEmitterPin(2);
    delay(500);
}

void readAndNormalizeSensors() {
    qtr.read(sensorValues);
    for (int i = 0; i < SensorCount; i++) {
        if (sensorValues[i] < calibratedMin[i]) normalizedValues[i] = 0;
        else if (sensorValues[i] > calibratedMax[i]) normalizedValues[i] = 1000;
        else normalizedValues[i] = ((sensorValues[i] - calibratedMin[i]) * 1000) / (calibratedMax[i] - calibratedMin[i]);
    }
}

void setupMotors() {
    pinMode(Motor1_forward, OUTPUT); pinMode(Motor1_backward, OUTPUT);
    pinMode(Motor2_forward, OUTPUT); pinMode(Motor2_backward, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
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
   double adjustment = KP * error + KD * (error - last_error);
   last_error = error;
   controlMotors(constrain(motor_speed + adjustment, 0, 150), constrain(motor_speed - adjustment, 0, 150)-20);
}

void back_linefollowing() {
    bool leftSensor = !digitalRead(back_Ir[1]);
    bool rightSensor = !digitalRead(back_Ir[2]);
    if (leftSensor && rightSensor) controlMotors(-150, -165);
    else if (leftSensor) controlMotors(-150,-80);
    else if (rightSensor) controlMotors(-80, -150);
    else controlMotors(-150, -165);
}

void read_grid() {
    for (uint8_t i = 0; i < 9; i++) Ir_values[i] = digitalRead(irSensorPins[i]);
    int sum = 0;
    for (uint8_t i = 0; i < 9; i++) sum += Ir_values[i];
    for (uint8_t i = 0; i < 9; i++) Ir_values[i] = !Ir_values[i];
    detectGoalsFromIRSensors();
    if (sum != 3) {
        moveForward(4); controlMotors(-100,-100); delay(50);
        moveBackward(6); controlMotors(100,100); delay(50);
        controlMotors(0,0); delay(500); read_grid();
    }
}

void TurnRight1(int targetTicks) {
    moveForward(5); controlMotors(-255, -255); delay(130);
    left_pulse_count = 0; right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) controlMotors(255, -255);
    controlMotors(-255, 255); delay(100); controlMotors(0, 0); delay(10);
}
void TurnLeft(int targetTicks) {
    moveForward(5); controlMotors(-255, -255); delay(130);
    left_pulse_count = 0; right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) controlMotors(-180, 150);
    controlMotors(180, -180); delay(80); controlMotors(0, 0); delay(10);
}
void TurnLeft1(int targetTicks) { TurnLeft(targetTicks); } 
void U_turn(int targetTicks) {
    left_pulse_count = 0; right_pulse_count = 0;
    while (right_pulse_count < targetTicks || left_pulse_count < targetTicks) controlMotors(-200, 200);
    controlMotors(200, -200); delay(100); controlMotors(0, 0); delay(40);
}
void moveForward(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0; right_pulse_count = 0;
    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) pid_linefollowing();
    controlMotors(0, 0);
}
void moveForward1(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0; right_pulse_count = 0;
    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) pid_linefollowing1();
    controlMotors(0, 0);
}
void moveBackward(float distance_cm) {
    long target_pulses = (distance_cm * PPR * GEAR_RATIO) / (PI * WHEEL_DIAMETER);
    left_pulse_count = 0; right_pulse_count = 0;
    controlMotors(-100,0); delay(30);
    while (left_pulse_count < target_pulses || right_pulse_count < target_pulses) back_linefollowing();
    controlMotors(0, 0);
}
bool detectJunction() {
    static bool previousJunctionState = false;
    bool currentJunctionState = true;
    readAndNormalizeSensors();
    for (int i = 0; i < 8; i++) if (normalizedValues[i] < 700) { currentJunctionState = false; break; }
    if (currentJunctionState && !previousJunctionState) { previousJunctionState = currentJunctionState; return true; }
    previousJunctionState = currentJunctionState;
    return false;
}

void rightEncoder() { right_pulse_count++; }
void leftEncoder() { left_pulse_count++; }

void detectColor() {
    uint16_t r, g, b, c;
    bool validColorDetected = false;
    while (!validColorDetected) {
        tcs.getRawData(&r, &g, &b, &c);
        if (c < MIN_BRIGHTNESS) { delay(500); continue; }
        if (r > g && r > b) { detectedColor = RED; specialGoal = {1, 7}; validColorDetected = true; }
        else if (g > r && g > b) { 
            detectedColor = GREEN; 
            specialGoal = (!firstBlueOrGreenDetected) ? (Point){1, 1} : (Point){5, 1}; 
            firstBlueOrGreenDetected = true; validColorDetected = true; 
        }
        else if (b > g && b > r) { 
            detectedColor = BLUE; 
            specialGoal = (!firstBlueOrGreenDetected) ? (Point){1, 1} : (Point){5, 1}; 
            firstBlueOrGreenDetected = true; validColorDetected = true; 
        }
        else { delay(500); }
    }
}

void detectGoalsFromIRSensors() {
    int detectedCount = 0;
    for (int i = 0; i < 9; i++) {
        if (digitalRead(irSensorPins[i]) == HIGH && detectedCount < 3) mainGoals[detectedCount++] = irSensorMap[i];
    }
    if (detectedCount > 0) {
        goalsDetected = true; totalMainGoals = detectedCount;
        for (int i = 0; i < 3; i++) visitedGoals[i] = false;
    }
}

void logState(Action action) {
    Serial.print("Pos: "); Serial.print(currentPos.x); Serial.print(","); Serial.print(currentPos.y);
    Serial.print(" Act: "); Serial.println(action);
}

void drop() { for (int angle = 60; angle <= 140; angle++) { myservo.write(angle); delay(5); } currentAngle = 140; }
void drop1() { for (int angle = 60; angle <= 140; angle++) { myservo.write(angle); delay(10); } currentAngle = 140; }
void lift() { for (int angle = 140; angle >= 60; angle--) { myservo.write(angle); delay(10); } currentAngle = 60; }

void executeInitialPath() {
    int junctionCounter = 0;
    while (junctionCounter < 5) {
        if (detectJunction()) {
            junctionCounter++;
            if (junctionCounter == 2) { TurnLeft1(175); moveBackward(18); read_grid(); pid_linefollowing(); }
            else if (junctionCounter == 3) { TurnRight1(200); }
            else if (junctionCounter == 5) { TurnLeft(190); }
        } else pid_linefollowing();
    }
    controlMotors(0,0);
}

void setup() {
    Serial.begin(9600);
    setupQTR(); setupMotors();
    myservo.attach(12); myservo.write(currentAngle);
    pinMode(IR_TURN_SENSOR_PIN, INPUT);
    for (int i = 0; i < 5; i++) pinMode(back_Ir[i], INPUT);
    for (int i = 0; i < 9; i++) pinMode(irSensorPins[i], INPUT);
    pinMode(ENCA_R, INPUT); pinMode(ENCA_L, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_R), rightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_L), leftEncoder, RISING);
    if (tcs.begin()) Serial.println("Color Sensor Ready");
    executeInitialPath();
}

void loop() {
    switch (robotState) {
        case MOVING_TO_MAIN_GOAL:
            if (currentMainGoalIndex >= totalMainGoals) { robotState = MOVING_TO_PARKING; break; }
            if (itemCount == 0) {
                updateGridForPathPlanning(mainGoals[currentMainGoalIndex]);
                // CALL STPO Algorithm with parameters (Start, End, StartDir, TurnPenalty, SegmentTime)
                dijkstraSTPO(currentPos, mainGoals[currentMainGoalIndex], currentDir, TIME_TURN_90, TIME_SEGMENT);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                controlMotors(-255, -255); delay(40); controlMotors(0,0);
                robotState = APPROACHING_MAIN_GOAL;
            }
            break;

        case APPROACHING_MAIN_GOAL:
            pid_linefollowing();
            if (digitalRead(IR_TURN_SENSOR_PIN) == LOW) {
                controlMotors(-220, -255); delay(130); controlMotors(0, 0); delay(10);
                robotState = DETECTING_COLOR;
            }
            break;

        case DETECTING_COLOR:
            moveForward(4); lift(); controlMotors(0,0);
            detectColor();
            visitedGoals[currentMainGoalIndex] = true;     
            updateGridForPathPlanning(specialGoal);
            
            dijkstraSTPO(currentPos, specialGoal, currentDir, TIME_TURN_90, TIME_SEGMENT);
            
            resetPathFlag = true;
            robotState = MOVING_TO_SPECIAL_GOAL;
            break;

        case MOVING_TO_SPECIAL_GOAL:
            if (itemCount == 0) {
                updateGridForPathPlanning(specialGoal);
                dijkstraSTPO(currentPos, specialGoal, currentDir, TIME_TURN_90, TIME_SEGMENT);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                controlMotors(-220,-255); delay(110); controlMotors(0,0);
                robotState = WAITING_AT_SPECIAL_GOAL;
                goalReachedTime = millis();
            }
            break;

        case WAITING_AT_SPECIAL_GOAL:
            if (detectedColor == BLUE || detectedColor == GREEN) {
                if (specialGoal.x == 1 && specialGoal.y == 1) {
                    moveForward(6); drop(); moveBackward(8);
                    currentDir = NORTH; currentPos = {1, 2};
                } else {
                    controlMotors(-150,-150); delay(30); moveForward1(6); drop(); moveBackward(8);
                    currentDir = NORTH; currentPos = {5, 2};
                }
            } else if (detectedColor == RED) {
                drop(); moveForward(21); moveBackward(21);
                currentDir = SOUTH; currentPos = {1, 6};
            }
            controlMotors(0, 0); 
            detectedColor = NO_COLOR; currentMainGoalIndex++; resetPathFlag = false;
            robotState = (currentMainGoalIndex < totalMainGoals) ? MOVING_TO_MAIN_GOAL : MOVING_TO_PARKING;
            break;

        case MOVING_TO_PARKING:
            if (itemCount == 0) {
                updateGridForPathPlanning(parking);
                dijkstraSTPO(currentPos, parking, currentDir, TIME_TURN_90, TIME_SEGMENT);
                resetPathFlag = true;
            }
            if (followPathToGoal()) {
                controlMotors(100, 100); delay(400); controlMotors(-200, -200); delay(100); controlMotors(0,0);
                while (1);
            }
            break;
    }
}