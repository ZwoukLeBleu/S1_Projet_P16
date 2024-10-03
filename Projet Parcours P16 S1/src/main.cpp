#include <Arduino.h>
#include <LibRobus.h>

#define MAZE_X 7
#define MAZE_Y 21

int maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

struct Robot {
    int x;           
    int y;            
    float speed;   
    int direction;    // 0: Haut, 1: Droite, 2: Bas, 3: Gauche)
};

Robot robot = {1, 1, 0.7, 1};
bool visited[MAZE_X][MAZE_Y] = {false};

void setup() {
    BoardInit();
}

void Stop() {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

long PID(long PreviousTime, float TargetSpeed) {
    float pid = 0;
    float Kp = 0.001;
    float Ki = 1;
    float Erreur = 0;
    float sec = 7000;
    unsigned int CT = 50;
    long CurrentTime = millis();
    unsigned int TimeSample = CurrentTime - PreviousTime;

    if (TimeSample >= CT) {
        int encodeur_0 = ENCODER_Read(0);
        int encodeur_1 = ENCODER_Read(1);
        Erreur = encodeur_0 - encodeur_1;
        pid = (Erreur * Kp) + ((Erreur * Ki) / sec) + TargetSpeed;
        PreviousTime = CurrentTime;
        MOTOR_SetSpeed(1, pid);
    }
    return PreviousTime;
}

void Left(int AngleG) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    AngleG = -AngleG;
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0.4);
    unsigned int dm = ((120 * AngleG) / 360);
    float cycle = (3200L * dm) / 23.9389;

    while (ENCODER_Read(1) < cycle) {}
    Stop();
}

void Right(int AngleD) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, 0.4);
    MOTOR_SetSpeed(1, 0);
    unsigned int dm = ((120 * AngleD) / 360);
    float cycle = (3200L * dm) / 23.9389;

    while (ENCODER_Read(0) < cycle) {}
    Stop();
}

void Backward(int DistanceR) {
    MOTOR_SetSpeed(0, -0.34);
    MOTOR_SetSpeed(1, -0.34);
    while (ENCODER_Read(0) >= DistanceR && ENCODER_Read(1) >= DistanceR) {}
    Stop();
}

void Foward(int DistanceA) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    float PreviousTime = 0;
    float speed0 = 0;
    float speed1 = robot.speed;
    float distance = DistanceA * 3200L / 24;

    float speedMin = 0.2;
    float speedMax = 0.5;
    float a = 0;
    float b = 0;

    unsigned long startTime = millis();

    while (true) {
        if (speed0 < speed1 && (ENCODER_Read(0) / distance) <= 0.4) {
            speed0 += 0.01;
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed0);
            delay(5);
        } else {
            if ((ENCODER_Read(0) / distance) >= 0.6) {
                a = (speedMin - speedMax) / (1 - 0.6);
                b = speedMin - (a * 1);
                speed0 = (a * (ENCODER_Read(0) / distance) + b);
                MOTOR_SetSpeed(0, speed0);
                MOTOR_SetSpeed(1, speed0);
            }
        }

        if (ENCODER_Read(0) >= distance && ENCODER_Read(1) >= distance) {
            Stop();
            break;
        }
    }
}

void moveTo(int newX, int newY) {
    robot.x = newX;
    robot.y = newY;
    visited[newX][newY] = true;
}

bool canMove(int x, int y) {
    return x >= 0 && x < MAZE_X && y >= 0 && y < MAZE_Y && maze[x][y] == 0 && !visited[x][y];
}

void exploreMaze() {
    int newX = robot.x, newY = robot.y;
    
    if (robot.direction == 0) newX--;
    else if (robot.direction == 1) newY++; 
    else if (robot.direction == 2) newX++; 
    else if (robot.direction == 3) newY--; 

    if (canMove(newX, newY)) {
        Foward(10);
        moveTo(newX, newY);
        return;
    }

    Right(90);
    robot.direction = (robot.direction + 1) % 4;

    exploreMaze();
}

void loop() {
    exploreMaze();
    delay(500);
}