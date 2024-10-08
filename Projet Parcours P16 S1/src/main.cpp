#include <Arduino.h>
#include <LibRobus.h>
#include <math.h>
#include <stdbool.h>

// Constantes pour les dimensions du labyrinthe
#define MAZE_X 7
#define MAZE_Y 21
#define WHEEL_DIAMETER 9.0f // en cm
#define ENCODER_COUNT 3200
#define PULSE_PER_CM (ENCODER_COUNT / (PI * WHEEL_DIAMETER) * 0.9f)
#define TURN_PULSES 1925
#define IR_OFF 900
#define IR_ON 100

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

    int value; // Distance en unités ou angle en degrés
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

// Struct pour l'état du PID
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

        error = encodeur_0 - encodeur_1;
        pid = (error * pidController.Kp) + ((error * pidController.Ki) / pidController.sec) + targetSpeed;

        previousTime = currentTime;

        MOTOR_SetSpeed(motor1, pid);
    }
    return targetSpeed;
}

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
inline void Stop() {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

// Fonction pour tourner à gauche
void TurnLeft(int angle = 90) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, -0.2f);
    MOTOR_SetSpeed(1, 0.2f);
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    Stop();

    robot.direction = static_cast<Direction>((robot.direction + 3) % 4);
    Serial.print("Direction après tourner à gauche : ");
    Serial.println(robot.direction);
}

// Fonction pour tourner à droite
void TurnRight(int angle = 90) {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, 0.2f);
    MOTOR_SetSpeed(1, -0.2f);
    while (ENCODER_Read(0) < TURN_PULSES && ENCODER_Read(1) < TURN_PULSES) { }
    Stop();

    robot.direction = static_cast<Direction>((robot.direction + 1) % 4);
    Serial.print("Direction après tourner à droite : ");
    Serial.println(robot.direction);
}

// Fonction pour tourner de 180 degrés
void TurnBack() {
    TurnLeft(180);
    TurnLeft(180);
}

// Fonction pour vérifier la présence d'un mur
int wallCheck() {
    int red = analogRead(A0); 
    int green = analogRead(A1);

    if (red < IR_ON && green < IR_ON) {
        return 1; // Mur détecté
    } else if (red > IR_OFF && green > IR_OFF) {
        return 0; // Pas de mur
    }
    return -1; // erreur x
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

// Fonction pour avancer
void MoveForward() {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    float targetSpeed = robot.speed;
    unsigned long previousTime = millis();

    float currentDistance = 0.0f;
    float lastUpdateDistance = 0.0f;

    // Initial accélération
    float speed0 = 0.0f;
    float speed1 = 0.0f;

    while (true) {
        unsigned long currentTime = millis();
        unsigned int deltaTime = currentTime - previousTime;

        if (deltaTime >= pidController.CT) {
            int encoder0 = ENCODER_Read(0);
            int encoder1 = ENCODER_Read(1);

            // Calcul de la distance parcourue
            float distance = (encoder0 + encoder1) / (2.0f * PULSE_PER_CM);
            currentDistance += distance;

            PID(previousTime, targetSpeed, 0, 1);
            PID(previousTime, targetSpeed, 1, 0);

            previousTime = currentTime;

            // Réinitialiser les encodeurs
            ENCODER_Reset(0);
            ENCODER_Reset(1);
        }

        // Accélération progressive
        if (speed0 < targetSpeed && speed1 < targetSpeed) {
            speed0 += 0.01f;
            speed1 += 0.01f;
            MOTOR_SetSpeed(0, speed0);
            MOTOR_SetSpeed(1, speed1);
        }

        // Détection de mur
        int wall = wallCheck();
        if (wall == 1) { // Mur détecté
            int nmbTiles = static_cast<int>((currentDistance - lastUpdateDistance) / 25.0f);

            for (int i = 0; i < nmbTiles; ++i) {
                Position newPos = robot.pos;
                switch (robot.direction) {
                    case HAUT:    newPos.x -= 1; break;
                    case DROITE:  newPos.y += 1; break;
                    case BAS:     newPos.x += 1; break;
                    case GAUCHE:  newPos.y -= 1; break;
                }

                // Vérifier les limites du labyrinthe
                if (newPos.x < 0 || newPos.x >= MAZE_X || newPos.y < 0 || newPos.y >= MAZE_Y) {
                    Serial.println("Overflow des bornes");
                    Stop();
                    return;
                }

                // Vérifier s'il y a un mur dans la nouvelle position
                if (maze[newPos.x][newPos.y] == 1) {
                    Serial.println("Mur détecté");
                    Stop();
                    return;
                }

                // Mettre à jour la position et marquer la case comme visitée
                robot.pos = newPos;
                visited[newPos.x][newPos.y] = true;
                afficherLabyrinthe();

                Serial.print("Position mise à jour : (");
                Serial.print(robot.pos.x);
                Serial.print(", ");
                Serial.print(robot.pos.y);
                Serial.println(")");
            }

            lastUpdateDistance += nmbTiles * 25.0f;

            Stop();
            ENCODER_Reset(0);
            ENCODER_Reset(1);
            break;
        }

        delay(5);
    }
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

// Met à jour la position du robot et marque la case comme visitée
void moveTo(const Position& newPos) {
    robot.pos = newPos;
    visited[newPos.x][newPos.y] = true;
}

// Vérifie si le robot peut se déplacer vers une position donnée
bool canMove(int x, int y) {
    return x >= 0 && x < MAZE_X && y >= 0 && y < MAZE_Y &&
           maze[x][y] == 0 && !visited[x][y];
}

// Implémentation de l'exploration du labyrinthe avec backtracking
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
    }
}

// Initialise les paramètres du robot
void initRobot() {
    robot.pos = {1, 1}; // Position de départ
    robot.speed = 0.35f; // Vitesse cible
    robot.direction = HAUT; // Direction initiale
}

// Configuration initiale
void setup() {
    BoardInit();             // Initialiser le plateau de contrôle Robus
    initRobot();             // Initialiser les paramètres du robot
}

void loop() {
    static bool hasExplored = false;
    if (!hasExplored) {
        exploreMaze();
        hasExplored = true;
    }
}
