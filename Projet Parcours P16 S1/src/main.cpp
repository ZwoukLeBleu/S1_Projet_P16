#include <Arduino.h>
#include <LibRobus.h>
#include <math.h>
#include <stdbool.h>

// Constantes pour les dimensions du labyrinthe
#define MAZE_X 7
#define MAZE_Y 21
#define WHEEL_DIAMETER 7.62f 
#define ENCODER_COUNT 3200
#define PULSE_PER_CM (ENCODER_COUNT / (PI * WHEEL_DIAMETER))
#define TURN_PULSES 1925
#define IR_OFF 900
#define IR_ON 200
#define PIN_RED A0
#define PIN_GREEN A1
#define PIN_AMBIENT A2
#define PIN_5KHZ A3
#define KHZ_5_ON -150
#define KHZ_1_ON -170

// Énumération pour les directions
enum Direction {
    HAUT = 0,
    DROITE = 1,
    BAS = 2,
    GAUCHE = 3
};

// Struct pour représenter une position
struct Position {
    int x;
    int y;
};

// Struct pour représenter le robot
struct Robot {
    float speed;
    Position pos;
    Direction direction;
} robot;

// Struct pour représenter un mouvement
struct Movement {
    enum Type {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    } type;

    int value;
};

// Matrice de visite
bool visited[MAZE_X][MAZE_Y] = {false};

// 1 = mur, 0 = espace libre
uint8_t maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} 
};
/*uint8_t maze[MAZE_X][MAZE_Y] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};*/

// Struct pour l'état du PID
struct PIDController {
    const float Kp;
    const float Ki;
    const unsigned int CT;

    PIDController() : Kp(0.002f), Ki(0.01f), CT(50) {}
} pidController;

// Structure pour maintenir l'état du PID
struct PIDState {
    float integral;
} pidState = {0.0f};

// Déclaration des fonctions
void Stop();
int wallCheck();
int whistleCheck();
void TurnBack();
void initRobot();
void exploreMaze();
void MoveForward();
void afficherLabyrinthe();
bool canMove(int x, int y);
void TurnLeft(int angle = 90);
void TurnRight(int angle = 90);
void moveTo(const Position& newPos);
inline void performMovement(const Movement& movement);
long computePID(long previousTime, float targetSpeed, int motorMaster, int motorSlave);

#define STACK_SIZE (MAZE_X * MAZE_Y)
struct Stack {
    Position items[STACK_SIZE];
    int top;

    Stack() : top(-1) {}

    bool isEmpty() const {
        return top == -1;
    }

    bool push(const Position& pos) {
        if (top >= STACK_SIZE - 1) return false; 
        items[++top] = pos;
        return true;
    }

    bool pop(Position& pos) {
        if (isEmpty()) return false;
        pos = items[top--];
        return true;
    }
} pathStack;

// Fonction pour arrêter le robot
inline void  Stop() {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

// Fonction pour tourner à gauche
void TurnLeft(int angle) {
    /*ENCODER_Reset(0);
    ENCODER_Reset(1);

    float targetSpeed = 0.2f;
    unsigned long previousTime = millis();

    long pulsesNeeded = (TURN_PULSES * angle) / 90;

    while ((ENCODER_Read(0) < pulsesNeeded) || (ENCODER_Read(1) < pulsesNeeded)) {
        previousTime = computePID(previousTime, targetSpeed, 0, 1);
        MOTOR_SetSpeed(0, -targetSpeed);
        MOTOR_SetSpeed(1, targetSpeed);
    }
    Stop();

    // Mise à jour de la direction
    robot.direction = static_cast<Direction>((robot.direction + 3) % 4);
    Serial.print("Direction après tourner à gauche : ");
    Serial.println(robot.direction);*/
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, -0.2f); 
    MOTOR_SetSpeed(1, 0.2f); 
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    
    Stop();

    robot.direction = static_cast<Direction>((robot.direction + 3) % 4);
}

// Fonction pour tourner à droite
void TurnRight(int angle) {
    /*ENCODER_Reset(0);
    ENCODER_Reset(1);

    float targetSpeed = 0.2f;
    unsigned long previousTime = millis();

    long pulsesNeeded = (TURN_PULSES * angle) / 90; 

    while ((ENCODER_Read(0) < pulsesNeeded) || (ENCODER_Read(1) < pulsesNeeded)) {
        previousTime = computePID(previousTime, targetSpeed, 0, 1);
        MOTOR_SetSpeed(0, targetSpeed);
        MOTOR_SetSpeed(1, -targetSpeed);
    }
    Stop();

    robot.direction = static_cast<Direction>((robot.direction + 1) % 4);
    Serial.print("Direction après tourner à droite : ");
    Serial.println(robot.direction);*/
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, 0.2f); 
    MOTOR_SetSpeed(1, -0.2f); 
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    
    Stop();
    robot.direction = static_cast<Direction>((robot.direction + 1) % 4);
}

// Fonction pour tourner de 180 degrés
void TurnBack() {
    TurnLeft(180);
}

// Fonction pour vérifier la présence d'un mur
int wallCheck() {
    int red = analogRead(PIN_RED); 
    int green = analogRead(PIN_GREEN);

    if (red < IR_ON && green < IR_ON) {
        return 1; // Mur détecté
    } else if (red > IR_OFF && green > IR_OFF) {
        return 0; // Pas de mur
    }
    return -1; // Erreur
}

int whistleCheck(){
    if(analogRead(PIN_5KHZ) - analogRead(PIN_AMBIENT) >= KHZ_5_ON){ return 1; }
    return 0;
}

// Fonction pour afficher l'état du labyrinthe
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

// Fonction pour mettre à jour la position du robot et marquer la case comme visitée
void moveTo(const Position& newPos) {
    robot.pos = newPos;
    visited[newPos.x][newPos.y] = true;
    pathStack.push(newPos);
}

// Vérifie si le robot peut se déplacer vers une position donnée
bool canMove(int x, int y) {
    return x >= 0 && x < MAZE_X && y >= 0 && y < MAZE_Y &&
           maze[x][y] == 0 && !visited[x][y];
}

// Fonction pour exécuter un mouvement
inline void performMovement(const Movement& movement) {
    delay(15);
    switch (movement.type) {
        case Movement::FORWARD:
            MoveForward();
            break;
        case Movement::TURN_LEFT:
            TurnLeft(movement.value);
            break;
        case Movement::TURN_RIGHT:
            TurnRight(movement.value);
            break;
        case Movement::TURN_BACK:
            TurnBack();
            break;
    }
}

// Fonction pour avancer avec régulation PID
void MoveForward() {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    float targetSpeed = robot.speed;
    unsigned long previousTime = millis();

    float currentDistance = 0.0f;
    float lastUpdateDistance = 0.0f;

    float speedMaster = 0.0f;
    float speedSlave = 0.0f;

    while (true) {
        
        int nbrTiles = 0;
        if (currentDistance >= 25.0f) {
            currentDistance -= 25.0f;
            //nbrTiles++;
            switch (robot.direction) {
                    case HAUT:    robot.pos.x -= 1; break;
                    case DROITE:  robot.pos.y += 1; break;
                    case BAS:     robot.pos.x += 1; break;
                    case GAUCHE:  robot.pos.y -= 1; break;
                }
        }
        /*Serial.print("Nouvelle position : (");
        Serial.print(robot.pos.x);
        Serial.print(", ");
        Serial.print(robot.pos.y);
        Serial.println(")");
        Serial.println(maze[robot.pos.x][robot.pos.y]);*/

        
        // Détection de mur
        Position newPos = robot.pos;
        if (maze[newPos.x][newPos.y] == 1){
            Serial.println("Tape détecté");
            
            return;
        }
        if (wallCheck() == 1) { // Mur détecté
            
            //for (int i = 0; i < nbrTiles; ++i) {
                
            /*switch (robot.direction) {
                case HAUT:    newPos.x -= 1; break;
                case DROITE:  newPos.y += 1; break;
                case BAS:     newPos.x += 1; break;
                case GAUCHE:  newPos.y -= 1; break;
            }*/

            // Vérifier les limites du labyrinthe
            /*if (newPos.x < 0 || newPos.x >= MAZE_X || newPos.y < 0 || newPos.y >= MAZE_Y) {
                Serial.println("Overflow des bornes");
                Stop();
                return;
            }*/

            // Vérifier s'il y a un mur dans la nouvelle position

            // Mettre à jour la position et marquer la case comme visitée
            maze[newPos.x][newPos.y] = 1;
            moveTo(newPos);
            afficherLabyrinthe();
            Stop();

            Serial.print("Position mise à jour : (");
            Serial.print(robot.pos.x);
            Serial.print(", ");
            Serial.print(robot.pos.y);
            Serial.println(")");
            //}

            lastUpdateDistance = currentDistance;  // Mise à jour de la distance pour calculer les cases traversées
            break;
            
        }
        
        unsigned long currentTime = millis();
        unsigned int deltaTime = currentTime - previousTime;

        if (deltaTime >= pidController.CT) {
            previousTime = computePID(previousTime, targetSpeed, 0, 1); // 0: moteur maître, 1: moteur esclave

            int encoder0 = ENCODER_Read(0);
            int encoder1 = ENCODER_Read(1);
            float distance = (encoder0 + encoder1) / (2.0f * PULSE_PER_CM)*100*PI*2;
            
            currentDistance += distance;
            //Serial.print("Distance parcourue : ");
            //Serial.println(currentDistance);

            ENCODER_Reset(0);
            ENCODER_Reset(1);
        }

        if (speedMaster < targetSpeed && speedSlave < targetSpeed) {
            speedMaster += 0.01f;
            speedSlave += 0.01f;
            MOTOR_SetSpeed(0, speedMaster);
            MOTOR_SetSpeed(1, speedSlave);
        }

        delay(50);
    }
    
}

// Fonction pour l'exploration du labyrinthe avec backtracking
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
                    movement = { Movement::FORWARD, 0 };
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
            Position backPos;
            if (pathStack.pop(backPos)) {
                int dx = backPos.x - robot.pos.x;
                int dy = backPos.y - robot.pos.y;
                Direction backDir;

                if (dx == 1) backDir = BAS;
                else if (dx == -1) backDir = HAUT;
                else if (dy == 1) backDir = DROITE;
                else if (dy == -1) backDir = GAUCHE;

                // Tourner vers la direction inverse
                Direction desiredDir = static_cast<Direction>((backDir + 2) % 4);
                int turnDifference = (desiredDir - robot.direction + 4) % 4;

                if (turnDifference == 1) {
                    performMovement({ Movement::TURN_RIGHT, 90 });
                } else if (turnDifference == 3) {
                    performMovement({ Movement::TURN_LEFT, 90 });
                } else if (turnDifference == 2) {
                    performMovement({ Movement::TURN_BACK, 180 });
                }

                performMovement({ Movement::FORWARD, 0 });
                moveTo(backPos);
                afficherLabyrinthe();
            } else {
                Serial.println("Exploration terminée ou impasse atteinte.");
                Stop();
                break;
            }
        }
    }
}

// Initialise les paramètres du robot
void initRobot() {
    robot.pos = {3, 1}; // Position de départ
    robot.speed = 0.35f; // Vitesse cible
    robot.direction = DROITE; // Direction initiale
}

// Fonction du PID pour réguler la vitesse des moteurs
long computePID(long previousTime, float targetSpeed, int motorMaster, int motorSlave) {
    long currentTime = millis();
    unsigned int deltaTime = currentTime - previousTime;

    if (deltaTime >= pidController.CT) {
        // Lecture des encodeurs
        int encoderMaster = ENCODER_Read(motorMaster);
        int encoderSlave = ENCODER_Read(motorSlave);

        // Calcul de l'erreur entre les deux moteurs
        float error = encoderMaster - encoderSlave;

        // Accumulation de l'erreur intégrale
        pidState.integral += error * (deltaTime / 1000.0f);

        // Calcul de la sortie PID
        float output = (pidController.Kp * error) + (pidController.Ki * pidState.integral) + targetSpeed;

        // Régulation de la vitesse du moteur esclave
        MOTOR_SetSpeed(motorSlave, output);

        // Réinitialisation des encodeurs après lecture
        ENCODER_Reset(motorMaster);
        ENCODER_Reset(motorSlave);

        // Mise à jour du temps précédent
        previousTime = currentTime;
    }

    return previousTime;
}

// Configuration initiale
void setup() {
    BoardInit();              // Initialiser le plateau de contrôle Robus
    initRobot();              // Initialiser les paramètres du robot
    afficherLabyrinthe();     // Afficher l'état initial du labyrinthe
}

// Boucle de fonctionnement principale
void loop() {
    static bool hasExplored = false;
    if (!hasExplored ) {//&& whistleCheck()
        exploreMaze();
        hasExplored = true;
    }
   /*Serial.print("RED : ");
    Serial.println(analogRead(PIN_RED));
    Serial.print("GREEN : ");
    Serial.println(analogRead(PIN_GREEN));*/
    //Serial.println(wallCheck());
}