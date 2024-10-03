#include <Arduino.h>
#include <LibRobus.h>

// Constantes pour les dimensions du labyrinthe
#define MAZE_X 7
#define MAZE_Y 21
#define ROTOR_TARGET 3200

bool visited[MAZE_X][MAZE_Y] = {false};

// Définition du labyrinthe : 1 = mur, 0 = espace libre
const uint8_t maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

// Énumération pour les directions
enum Direction {
    HAUT = 0,
    DROITE = 1,
    BAS = 2,
    GAUCHE = 3
};

// Struct pour représenter une position
struct Position {
    uint8_t x;
    uint8_t y;
};

// Struct pour représenter le robot
struct Robot {
    float speed;
    Position pos;
    Direction direction;
}robot;

// Struct pour représenter un mouvement
struct Movement {
    enum Type {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    } type;
    
    int value; // Distance en unités ou angle en degrés
};

inline void Stop() {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

struct PIDController {
    const float Kp;
    const float Ki;
    const float sec;
    const unsigned int CT;
    
    PIDController() : Kp(0.001f), Ki(1.0f), sec(7000.0f), CT(50) {}
} pidController;

long PID(long previousTime, float targetSpeed, int encodeurIndex) {
    float pid = 0;
    float error = 0;
    long currentTime = millis();
    unsigned int timeSample = currentTime - previousTime;

    if (timeSample >= pidController.CT) {
        int encodeur = ENCODER_Read(encodeurIndex);
        //int encodeur_1 = ENCODER_Read(1);

        Serial.print("Encodeur 0: ");
        Serial.println(encodeur);
        //Serial.print("Encodeur 1: ");
        //Serial.println(encodeur_1);

        error = encodeur - ROTOR_TARGET;
        pid = (error * pidController.Kp) + ((error * pidController.Ki) / pidController.sec) + targetSpeed;

        Serial.print("Erreur: ");
        Serial.println(error);
        Serial.print("PID: ");
        Serial.println(pid);

        previousTime = currentTime;

        MOTOR_SetSpeed(encodeurIndex, pid); 
    }
    return previousTime;
}

void TurnLeft(int angle) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0.4f);

    unsigned int dm = (120 * angle) / 360;
    float cycle = (3200.0f * dm) / 23.9389f;

    while (ENCODER_Read(1) < cycle) { }

    Stop();
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    robot.direction = static_cast<Direction>((robot.direction + 3) % 4);
}

void TurnRight(int angle) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, 0.4f);
    MOTOR_SetSpeed(1, 0);

    unsigned int dm = (120 * angle) / 360;
    float cycle = (3200.0f * dm) / 23.9389f;

    while (ENCODER_Read(0) < cycle) {}

    Stop();
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    robot.direction = static_cast<Direction>((robot.direction + 1) % 4);
}

void MoveForward(int distance) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    unsigned long previousTime = 0;
    float speed0 = 0.0f;
    float targetSpeed = robot.speed;
    float distanceCounts = distance * 3200.0f / 24.0f;

    const float speedMin = 0.2f;
    float speedMax = 0.0f;
    float a = 0.0f;
    float b = 0.0f;

    while (true) {
        float encoderRatio = static_cast<float>(ENCODER_Read(0)) / distanceCounts;

        if (speed0 <= targetSpeed && encoderRatio <= 0.4f) {
            speed0 += 0.01f;
            speedMax = speed0;
            delay(5); 
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed0);
        }

        previousTime = PID(previousTime, speed0, 0);
        previousTime = PID(previousTime, speed0, 1);

        if (ENCODER_Read(0) >= distanceCounts && ENCODER_Read(1) >= distanceCounts) {
            Stop();
            ENCODER_Reset(0);
            ENCODER_Reset(1);
            break;
        }

        if (encoderRatio >= 0.6f) {
            a = (speedMin - speedMax) / (1.0f - 0.6f);
            b = speedMin - (a * 1.0f);
            speed0 = a * encoderRatio + b;
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed0); 
        }
    }
}

inline void performMovement(const Movement& movement) {
    switch (movement.type) {
        case Movement::FORWARD:
            MoveForward(movement.value);
            break;
        case Movement::TURN_LEFT:
            TurnLeft(movement.value);
            break;
        case Movement::TURN_RIGHT:
            TurnRight(movement.value);
            break;
        case Movement::TURN_BACK:
            TurnLeft(180);
            break;
    }
}

inline void moveTo(const Position& newPos) {
    robot.pos = newPos;
    visited[newPos.x][newPos.y] = true;
}

inline bool canMove(int x, int y) {
    return x >= 0 && x < MAZE_X && y >= 0 && y < MAZE_Y &&
           maze[x][y] == 0 && !visited[x][y];
}

void exploreMaze() {
    while (true) {
        visited[robot.pos.x][robot.pos.y] = true;
        bool moved = false;

        for (uint8_t i = 0; i < 4; ++i) {
            uint8_t newDir = (robot.direction + i) % 4;
            Position newPos = robot.pos;

            switch (newDir) {
                case HAUT:    newPos.x -= 1; break;
                case DROITE:  newPos.y += 1; break;
                case BAS:     newPos.x += 1; break;
                case GAUCHE:  newPos.y -= 1; break;
            }

            if (canMove(newPos.x, newPos.y)) {
                Movement movement;
                if (newDir == robot.direction) {
                    movement = { Movement::FORWARD, 50 };
                } else if ((newDir - robot.direction + 4) % 4 == 1) {
                    movement = { Movement::TURN_RIGHT, 90 };
                } else if ((newDir - robot.direction + 4) % 4 == 3) {
                    movement = { Movement::TURN_LEFT, 90 };
                } else {
                    movement = { Movement::TURN_BACK, 0 };
                }

                performMovement(movement);
                moveTo(newPos);
                moved = true;
                break;
            }
        }

        if (!moved) {
            Stop();
            break;
        }
    }
}

void initRobot() {
    robot.pos = {1, 1};
    robot.direction = DROITE;
    robot.speed = 0.95f;
}

void setup() {
    BoardInit();
    initRobot();
}

void loop() {
    static bool hasExplored = false;
    /*
    if (!hasExplored) {
        exploreMaze();
        hasExplored = true;
    }
    */
   MoveForward(50);
}