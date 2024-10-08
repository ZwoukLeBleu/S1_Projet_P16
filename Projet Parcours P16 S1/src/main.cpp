#include <Arduino.h>
#include <LibRobus.h>

// Constantes pour les dimensions du labyrinthe
#define MAZE_X 7
#define MAZE_Y 21
#define PI 3.141592f
#define WHEEL_DIAMETER 7.62f
#define ENCODER_COUNT 3200
#define PULSE_PER_CM ENCODER_COUNT*0.83f / (PI * WHEEL_DIAMETER)
#define TURN_PULSES 1925
#define IR_OFF 900
#define IR_ON 100
#define PIN_RED 0
#define PIN_GREEN 1
#define PIN_AMBIENT 2
#define PIN_5KHZ 3
#define KHZ_5_ON -150
#define KHZ_1_ON -170

bool visited[MAZE_X][MAZE_Y] = {false};

// Définition du labyrinthe : 1 = mur, 0 = espace libre
/*uint8_t maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};*/
uint8_t maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},//start     end
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
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

int wallCheck() {
    int red = analogRead(PIN_RED);
    int green = analogRead(PIN_GREEN);
    if (red < IR_ON && green < IR_ON) {
        return 1; //True -> wall ahead
    } else if (red > IR_OFF && green > IR_OFF) {
        return 0; //False -> no wall ahead
    }
    return -1;
}

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

long PID(long previousTime, float targetSpeed, int motor1, int motor2) {
    float pid = 0;
    float error = 0;
    long currentTime = millis();
    unsigned int timeSample = currentTime - previousTime;

    ENCODER_Reset(0);
    ENCODER_Reset(1);

    if (timeSample >= pidController.CT) {
        int encodeur_0 = ENCODER_Read(motor1);
        int encodeur_1 = ENCODER_Read(motor2);

        /*Serial.print("Encodeur 0: ");
        Serial.println(encodeur_0);
        Serial.print("Encodeur 1: ");
        Serial.println(encodeur_1);
*/
        error = encodeur_0 - encodeur_1;
        pid = (error * pidController.Kp) + ((error * pidController.Ki) / pidController.sec) + targetSpeed;
/*
        Serial.print("Erreur: ");
        Serial.println(error);
        Serial.print("PID: ");
        Serial.println(pid);
*/
        previousTime = currentTime;

        MOTOR_SetSpeed(motor1, pid);
    }
    return targetSpeed;
}


void TurnLeft(int angle) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, -0.2f); 
    MOTOR_SetSpeed(1, 0.2f); 
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    
    Stop();

    robot.direction = static_cast<Direction>((robot.direction + 3) % 4);
}

void TurnRight(int angle) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, 0.2f); 
    MOTOR_SetSpeed(1, -0.2f); 
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    
    Stop();
    robot.direction = static_cast<Direction>((robot.direction + 1) % 4);
}

inline void moveTo(const Position& newPos) {
    robot.pos = newPos;
    visited[newPos.x][newPos.y] = true;
}

inline bool canMove(int x, int y) {
    return x >= 0 && x < MAZE_X && y >= 0 && y < MAZE_Y &&
           maze[x][y] == 0 && !visited[x][y];
}
/*void MoveForward() {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    float speed0 = 0.0f;
    float speed1 = 0.0f;
    float targetSpeed = robot.speed;
    float currentDistance = 0;
    
    int encoder0 = 0;
    int encoder1 = 0;
    while (true) {
        encoder0 = ENCODER_ReadReset(0);
        encoder1 = ENCODER_ReadReset(1);
        currentDistance += (encoder0 + encoder1) / (2.0f * PULSE_PER_CM);
        int t1 = PID(0, targetSpeed, 0, 1);
        int t2 = PID(0, targetSpeed, 1, 0);

        if (speed0 < targetSpeed && speed1 < targetSpeed) {
            speed0 += 0.01f;
            speed1 += 0.01f;
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed1);
        }

        if (wallCheck() == 1) {
            int nmbTiles = static_cast<int>(currentDistance / 25.0f);
            switch (robot.direction) {

                case HAUT:    robot.pos.x -= nmbTiles; break;
                case DROITE:  robot.pos.y += nmbTiles; break;
                case BAS:     robot.pos.x += nmbTiles; break;
                case GAUCHE:  robot.pos.y -= nmbTiles; break;
            }
            Position nextPos = robot.pos;
            switch (robot.direction) {
                case HAUT:    nextPos.x -= 1; break;
                case DROITE:  nextPos.y += 1; break;
                case BAS:     nextPos.x += 1; break;
                case GAUCHE:  nextPos.y -= 1; break;
            }
            maze[nextPos.x][nextPos.y] = 1; //

            Stop();
            ENCODER_Reset(0);
            ENCODER_Reset(1);
            break;
        }
        else {for (uint8_t i = 0; i < 4; ++i) {
            uint8_t newDir = (robot.direction + i) % 4;
            Position newPos = robot.pos;
            
            
            
            switch (newDir) {
                case HAUT:    newPos.x -= 1; break;
                case DROITE:  newPos.y += 1; break;
                case BAS:     newPos.x += 1; break;
                case GAUCHE:  newPos.y -= 1; break;
            }
           
            if (canMove(newPos.x, newPos.y) == 1) {
                Movement movement;
                if (newDir == robot.direction) {
                    movement = { Movement::FORWARD };
                } else if ((newDir - robot.direction + 4) % 4 == 1) {
                    movement = { Movement::TURN_RIGHT, 90 };
                } else if ((newDir - robot.direction + 4) % 4 == 3) {
                    movement = { Movement::TURN_LEFT, 90 };
                } else {
                    movement = { Movement::TURN_BACK, 0 };
                }

                //performMovement(movement);
                moveTo(newPos);
                //moved = true;
                break;
            }
        }
        }
        delay(5);
    }
}*/

void MoveForward(int distance) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    float speed0 = 0.0f;
    float speed1 = 0.0f;
    float targetSpeed = robot.speed;
    float currentDistance = 0;
    
    int encoder0 = 0;
    int encoder1 = 0;
    while (true) {
        encoder0 = ENCODER_Read(0);
        encoder1 = ENCODER_Read(1);
        currentDistance += (encoder0+encoder1) / (2.0f*PULSE_PER_CM);
        int t1 = PID(0, targetSpeed, 0, 1);
        int t2 = PID(0, targetSpeed, 1, 0);
        Serial.println(currentDistance);

        if (speed0 < targetSpeed && speed1 < targetSpeed) {
            speed0 += 0.01f;
            speed1 += 0.01f;
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed1);
        }

        if (currentDistance >= distance || wallCheck() == 1) {
            Stop();
            ENCODER_Reset(0);
            ENCODER_Reset(1);
            break;
        }
        /*Serial.print("Distance: ");
        Serial.println(currentDistance);
        Serial.print("Encoder 0: ");
        Serial.println(encoder0);
        Serial.print("Encoder 1: ");
        Serial.println(encoder1);
        delay(10);*/
    }
}
inline void performMovement(const Movement& movement) {
    delay(100);
    switch (movement.type) {
        case Movement::FORWARD:
            MoveForward(movement.value);
            //MoveForward();
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


void afficherLabyrinthe() {
    Serial.println("État du labyrinthe :");
    for (int i = 0; i < MAZE_X; ++i) {
        for (int j = 0; j < MAZE_Y; ++j) {
            if (robot.pos.x == i && robot.pos.y == j) {
                Serial.print("R "); // R pour Robot
            } else {
                Serial.print(maze[i][j] == 1 ? "# " : ". "); // # pour mur, . pour espace libre
            }
        }
        Serial.println();
    }
    Serial.println();
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
           
            if (canMove(newPos.x, newPos.y) == 1) {
                Movement movement;
                if (newDir == robot.direction) {
                    movement = { Movement::FORWARD, 25};
                } else if ((newDir - robot.direction + 4) % 4 == 1) {
                    movement = { Movement::TURN_RIGHT, 90 };
                } else if ((newDir - robot.direction + 4) % 4 == 3) {
                    movement = { Movement::TURN_LEFT, 90 };
                } else {
                    movement = { Movement::TURN_BACK, 0 };
                }

                performMovement(movement);
                moveTo(newPos);
                afficherLabyrinthe();
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
    robot.speed = 0.35f;
}

void setup() {
    BoardInit();
    initRobot();
}

int whistleCheck(){
    if(analogRead(PIN_5KHZ) - analogRead(PIN_AMBIENT) >= KHZ_5_ON){ return 1; }
    return 0;
}



void loop() {
    static bool hasExplored = false;
    if (!hasExplored && whistleCheck()) {
        exploreMaze();
        hasExplored = true;
    }
    //analogRead(0);
    /*Serial.print("RED: ");
    Serial.println(analogRead(0)); 
    Serial.print("GREEN: ");
    Serial.println(analogRead(1)); 

    delay(15);*/

    //MoveForward(WHEEL_DIAMETER*PI);
    //TurnRight(90);
    //Forward(75);

    //MOTOR_SetSpeed(0, 0.02f);
    //ENCODER_Read(0);
    //Serial.println(analogRead(PIN_5KHZ) - analogRead(PIN_AMBIENT));
    //MoveForward(WHEEL_DIAMETER*PI);
    //delay(500);


}