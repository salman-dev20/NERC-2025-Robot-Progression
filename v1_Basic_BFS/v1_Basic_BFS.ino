#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// ================== GLOBAL VARIABLES ==================
int currentAngle = 60;
float forwardDistance = 0;
bool hasPerformedAction = false;
unsigned long actionStartTime = 0;
const int ACTION_DELAY = 1000;
int delays = 400;
const int IR_TURN_SENSOR_PIN = 11; 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);



Servo myservo;
bool firstBlueOrGreenDetected = false;
bool resetPathFlag = false;  // ADDED: Path reset flag

// ================== SENSOR & MOTOR CONFIG ==================
const int OUTPUT_PIN = 53;
const int light_PIN = 11;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
unsigned int normalizedValues[SensorCount];
const unsigned int calibratedMin[SensorCount] = {42, 42, 41, 43, 41, 43, 42, 44};
const unsigned int calibratedMax[SensorCount] = {901, 898, 904, 903, 901, 911, 898, 920};

const int MIN_BRIGHTNESS = 500;

// Motor pins
const int Motor1_forward = 51;
const int Motor1_backward = 50;
const int Motor2_forward = 48;
const int Motor2_backward = 49;
const int ENA = 3;
const int ENB = 4;

// PID parameters
const double KP = 0.057;
const double KI = 0.0;
const double KD = 1.4;
const unsigned char motor_speed = 200;
const unsigned char motor_backspeed = 110;
double last_error = 0;
double Goal = 3500;
double integral = 0;

float KpTurn = 0.08;
float Kdturn = 0.1;
int errorTurn = 0, last_errorTurn = 0, derivativeYurn = 0;

// ================== ENCODER CONFIG ==================
#define PPR 11
#define WHEEL_DIAMETER 6.50
#define PI 3.14159265358979
#define GEAR_RATIO 30.0
#define BASE_WIDTH 203.2
const float GRID_STEP_DISTANCE = 10.0;

const int ENCA_R = 2;
const int ENCB_R = 9;
const int ENCA_L = 18;
const int ENCB_L = 8;

volatile long right_pulse_count = 0;
volatile long left_pulse_count = 0;

// ================== PATH PLANNING CONFIG ==================
#define ROWS 11
#define COLS 9
#define NODE_DISTANCE 100
#define MAX_PATH_LENGTH 50

enum Color { RED, GREEN, BLUE, NONE, NO_COLOR };
Color detectedColor = NO_COLOR;

enum Direction { NORTH, EAST, SOUTH, WEST };
enum Action { FORWARD, HARD_LEFT, HARD_RIGHT, U_TURN, INVALID };
enum RobotState { MOVING_TO_MAIN_GOAL, APPROACHING_MAIN_GOAL, DETECTING_COLOR, MOVING_TO_SPECIAL_GOAL, WAITING_AT_SPECIAL_GOAL, MOVING_TO_PARKING };
RobotState robotState = MOVING_TO_MAIN_GOAL;
unsigned long goalReachedTime = 0;
int currentMainGoalIndex = 0;
int totalMainGoals = 3;

bool visitedGoals[3] = {false, false, false};

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

struct Point {
    byte x;
    byte y;
};

// ================== IR SENSOR CONFIG ==================
const int back_Ir[5] = {46, 45, 44, 43, 42}; 
bool Ir_values[9]; 
const int irSensorPins[9] = {40, 39, 38, 37, 36, 35, 34, 33, 32}; // Digital pins for IR sensors
Point irSensorMap[9] = {
    {4, 6},  {3, 6},   {2, 6},
    {4, 5},  {3, 5},   {2, 5},
    {4, 4},  {3, 4},   {2, 4}
};
Point mainGoals[3];
bool goalsDetected = false;
//={{2,6},{4,4},{2,4}}
Point currentPos = {5, 9};
Direction currentDir = NORTH;
Point parking = {7,3};
Point specialGoal;

Point pathQueue[MAX_PATH_LENGTH];
int queueFront = 0;
int queueRear = -1;
int itemCount = 0;

struct BFSPoint {
    byte x;
    byte y;
    int depth;
};
BFSPoint bfsQueue[MAX_PATH_LENGTH];
int bfsFront = 0;
int bfsRear = 0;
bool bfsVisited[ROWS][COLS];
Point parent[ROWS][COLS];

// ================== FUNCTION IMPLEMENTATIONS ==================
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

void bfsInitialize(Point start, Point end) {
    bfsFront = 0;
    bfsRear = 0;
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLS; x++) {
            bfsVisited[y][x] = false;
            parent[y][x] = {-1, -1};
        }
    }

    if (start.x == end.x && start.y == end.y) {
        itemCount = 0;
        return;
    }

    bfsQueue[bfsRear++] = {start.x, start.y, 0};
    bfsVisited[start.y][start.x] = true;
    parent[start.y][start.x] = {-1, -1};

    const int dirs[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
    while (bfsFront < bfsRear) {
        BFSPoint current = bfsQueue[bfsFront++];

        for (int i = 0; i < 4; i++) {
            byte newX = current.x + dirs[i][0];
            byte newY = current.y + dirs[i][1];
            if (newX >= 0 && newX < COLS && newY >= 0 && newY < ROWS &&
                !bfsVisited[newY][newX] && grid[newY][newX] == 0) {
                bfsVisited[newY][newX] = true;
                bfsQueue[bfsRear++] = {newX, newY, current.depth + 1};
                parent[newY][newX] = {current.x, current.y};
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

    if (!bfsVisited[end.y][end.x]) {
        Serial.println("No path to goal");
        return;
    }

    Point tempPath[MAX_PATH_LENGTH];
    int pathIndex = 0;
    Point current = end;

    while (current.x != start.x || current.y != start.y) {
        tempPath[pathIndex++] = current;
        Point prev = parent[current.y][current.x];
        if (prev.x == -1 && prev.y == -1) {
            Serial.println("Path reconstruction failed");
            return;
        }
        current = prev;
    }
    tempPath[pathIndex++] = start;

    for (int i = pathIndex - 2; i >= 0; i--) {
        pushToQueue(tempPath[i]);
    }
}

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
            TurnLeft(210);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 3) % 4);
            break;
        case HARD_RIGHT:
            TurnRight1(240);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 1) % 4);
            break;
        case U_TURN:
            U_turn(410);
            currentDir = static_cast<Direction>((static_cast<int>(currentDir) + 2) % 4);
            break;
        default: break;
    }
}

bool followPathToGoal() {
    static bool firstPathExecution = true;
    bool fakeJunctionTriggered = false;

    // Handle path reset
    if (resetPathFlag) {
        firstPathExecution = true;
        resetPathFlag = false;
    }

    // Only trigger fake junction if not coming from WAITING_AT_SPECIAL_GOAL
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

    rightMotorSpeed = constrain(rightMotorSpeed, 0, 223);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 223);

    controlMotors(leftMotorSpeed-18, rightMotorSpeed);
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
        controlMotors(-120, -125);
    }
    else if (leftSensor) {
        controlMotors(-105,-50);
    }
    else if (rightSensor) {
        controlMotors(-50, -110);
    }
    else {
        controlMotors(-120, -125);
    }
}

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

void TurnRight1(int targetTicks) {
    moveForward(7);
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
        controlMotors(-180, 180);
    }
    controlMotors(180, -180);
    delay(100);
    controlMotors(0, 0);
    delay(40);
}

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

void rightEncoder() { right_pulse_count++; }
void leftEncoder() { left_pulse_count++; }

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
            digitalWrite(OUTPUT_PIN, HIGH);
            delay(100);
            digitalWrite(OUTPUT_PIN, LOW);
            delay(100);
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
            digitalWrite(OUTPUT_PIN, HIGH);
            delay(100);
            digitalWrite(OUTPUT_PIN, LOW);
            delay(100);
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
            digitalWrite(OUTPUT_PIN, HIGH);
            delay(100);
            digitalWrite(OUTPUT_PIN, LOW);
            delay(100);
            validColorDetected = true;
        } 
        else {
            Serial.println("No clear color detected");
            delay(500);
        }
    }
    digitalWrite(light_PIN, LOW);
}

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
        delay(20);
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

void executeInitialPath() {
    int junctionCounter = 0;
    while (junctionCounter < 5) {
        if (detectJunction()) {
            junctionCounter++;
            Serial.print("Junction Count: ");
            Serial.println(junctionCounter);

            if (junctionCounter == 2) {
                controlMotors(-150,-150);
                delay(20);
                TurnLeft(170);
                //moveForward(3);
                moveBackward(19);
                //delay(100);
                read_grid();
                pid_linefollowing();
            } else if (junctionCounter == 3) {
                TurnRight1(200);
                
            } else if (junctionCounter == 5) {
                TurnLeft(215);
            }
        } else {
            pid_linefollowing();
        }
    }
    drop();
    controlMotors(0,0);
    Serial.println("Initial path completed. Following A* path now.");
}

// ================== CORE FUNCTIONS ==================
void setup() {
    Serial.begin(9600);

  
     
    setupQTR();
    setupMotors();
    myservo.attach(12);
    myservo.write(currentAngle);

    pinMode(IR_TURN_SENSOR_PIN, INPUT);
    for (int i = 0; i < 5; i++) {
        pinMode(back_Ir[i], INPUT);
    }
    for (int i = 0; i < 9; i++) {
        pinMode(irSensorPins[i], INPUT);
    }

    pinMode(ENCA_R, INPUT);
    pinMode(ENCA_L, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_R), rightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_L), leftEncoder, RISING);

    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);
    Serial.println("Output pin 53 initialized.");

    if (tcs.begin()) {
        Serial.println("Color sensor ready");
    } else {
        Serial.println("Sensor not found");
        while (1);
    }
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
                bfsInitialize(currentPos, mainGoals[currentMainGoalIndex]);
                reconstructPath(currentPos, mainGoals[currentMainGoalIndex]);
                resetPathFlag = true; // Trigger fake junction
            }
            if (followPathToGoal()) {
                Serial.println("Main goal path complete");
                controlMotors(-230, -230);
                delay(50);
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
            if (irState == LOW) { // IR sensor LOW = object detected
                Serial.println("Object detected!");
                controlMotors(-230, -230);
                delay(60);
                controlMotors(0, 0);
                digitalWrite(light_PIN, HIGH);
                delay(10);
                digitalWrite(light_PIN, LOW);
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
            bfsInitialize(currentPos, specialGoal);
            reconstructPath(currentPos, specialGoal);
          
            if (itemCount == 0) {
                Serial.println("Warning: No path to special goal, retrying path planning");
            } else {
                resetPathFlag = true; // Trigger fake junction for next state
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
                bfsInitialize(currentPos, specialGoal);
                reconstructPath(currentPos, specialGoal);
                resetPathFlag = true; // Ensure fake junction triggers
            }
            if (followPathToGoal()) {
                robotState = WAITING_AT_SPECIAL_GOAL;
                goalReachedTime = millis();
                Serial.println("Special goal reached");
            }
            break;

        case WAITING_AT_SPECIAL_GOAL:
        {
            static bool actionStarted = false;
            static unsigned long actionStartTime = 0;
            
            if (!actionStarted) {
                actionStartTime = millis();
                actionStarted = true;
                
                Serial.print("Executing action for color: ");
                switch (detectedColor) {
                    case BLUE:
                    case GREEN:
                    {
                        Serial.println(detectedColor == BLUE ? "BLUE" : "GREEN");
                        if (specialGoal.x == 1 && specialGoal.y == 1) {
                            moveForward(6);
                            drop();
                            controlMotors(0,0);
                            controlMotors(-90,-110);
                            delay(400);
                            //delay(1000);
                           // moveBackward(5);
                            currentPos = {1, 2};
                            currentDir = NORTH;
                        } else {
                            controlMotors(-150,-150);
                            delay(30);
                            controlMotors(0,0);
                            delay(500);
                            moveForward1(3);
                            drop1();
                            controlMotors(0,0);
                            controlMotors(-90,-110);
                            delay(400);
                            moveBackward(8);
                            //moveBackward(50);
                            currentPos = {5, 2};
                            currentDir = NORTH;
                        }
                        break;
                    }
                    case RED:
                    {
                        Serial.println("RED");
                        //moveBackward(2);
                        drop();
                        controlMotors(0,0);
                        moveForward(23);
                        moveBackward(32);
                        currentPos = {1, 6};
                        currentDir = SOUTH;
                        break;
                    }
                    default:
                        Serial.println("NO_COLOR");
                        break;
                }
                controlMotors(0, 0); // Stop motors before transition
            }
            
            if (millis() - actionStartTime >= 3000) {
                actionStarted = false;
                detectedColor = NO_COLOR;
                currentMainGoalIndex++;
                resetPathFlag = false; // Prevent fake junction in next state
                robotState = (currentMainGoalIndex < totalMainGoals) 
                    ? MOVING_TO_MAIN_GOAL : MOVING_TO_PARKING;
                Serial.println("Action completed, moving to next goal");
            }
            break;
        }
        case MOVING_TO_PARKING:
            if (itemCount == 0) {
                Serial.println("Planning path to parking");
                updateGridForPathPlanning(parking);
                bfsInitialize(currentPos, parking);
                reconstructPath(currentPos, parking);
                resetPathFlag = true; // Trigger fake junction
            }
            if (followPathToGoal()) {
                controlMotors(100, 100);
                delay(400);
                controlMotors(0,0);
                Serial.println("Parking reached");
                while (1);
            }
            break;
    }
}