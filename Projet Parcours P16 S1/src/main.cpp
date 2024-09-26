#include <stdio.h>
#include <LibRobus.h> 
#include <Arduino.h>
#include <stdbool.h>

#define MAZE_X 7
#define MAZE_Y 21

typedef struct {
    int posX;
    int posY;
    int direction;
    int etat;
    float vitesse;
} Robot;

Robot robot;

int maze[MAZE_X][MAZE_Y] = { // 1 = mur, 0 = vide
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, //0,0 = gauche
    {1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //debut - fin
    {1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

void setupRobot(Robot* robot) {
    robot->etat = 0;
    robot->vitesse = 0.37;
    robot->posX = 1;
    robot->posY = 1;
    robot->direction = 1;
}

void setup() {
    setupRobot(&robot);
    BoardInit();
}

void arret() {
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}

void avance(Robot* robot, int distance) {
  Serial.println("Avance");
  MOTOR_SetSpeed(RIGHT, robot->vitesse);
  MOTOR_SetSpeed(LEFT, robot->vitesse);
  delay(distance);
}

void recule(Robot* robot, int distance) {
  Serial.println("Recule");
  MOTOR_SetSpeed(RIGHT, robot->vitesse);
  MOTOR_SetSpeed(LEFT, robot->vitesse);
  delay(1000);
  arret();
}

void tourneDroit(Robot* robot) {
  Serial.println("Tourne à droite");
  MOTOR_SetSpeed(RIGHT, 0.5 * robot->vitesse);
  MOTOR_SetSpeed(LEFT, -0.5 * robot->vitesse);
  delay(1000);
  robot->direction = (robot->direction + 1) % 4;
}

void tourneGauche(Robot* robot) {
  Serial.println("Tourne à gauche");
  MOTOR_SetSpeed(RIGHT, -0.5 * robot->vitesse);
  MOTOR_SetSpeed(LEFT, 0.5 * robot->vitesse);
  delay(1000);
  robot->direction = (robot->direction + 3) % 4;
}

bool canMoveForward(Robot* robot) {
  int nextX = robot->posX, nextY = robot->posY;

  switch (robot->direction) {
      case 0: nextY++; break;
      case 1: nextX++; break;
      case 2: nextY--; break;
      case 3: nextX--; break;
  }

  if (nextX < 0 || nextX >= MAZE_Y || nextY < 0 || nextY >= MAZE_X) {
      return false; // Hors labyrinthe
  }

  if (maze[nextY][nextX] == 1) {
      return false; // Mur
  }

  return true;
}

bool canTurnRight(Robot* robot) {
  int nextX = robot->posX, nextY = robot->posY;
  int newDirection = (robot->direction + 1) % 4;

  switch (newDirection) {
      case 0: nextY++; break;
      case 1: nextX++; break;
      case 2: nextY--; break;
      case 3: nextX--; break;
  }

  if (nextX < 0 || nextX >= MAZE_Y || nextY < 0 || nextY >= MAZE_X) {
      return false; // Hors labyrinthe
  }

  if (maze[nextY][nextX] == 1) {
      return false; // Mur
  }

  return true;
}

bool canTurnLeft(Robot* robot) {
  int nextX = robot->posX, nextY = robot->posY;
  int newDirection = (robot->direction + 3) % 4;

  switch (newDirection) {
      case 0: nextY++; break;
      case 1: nextX++; break;
      case 2: nextY--; break;
      case 3: nextX--; break;
  }

  if (nextX < 0 || nextX >= MAZE_Y || nextY < 0 || nextY >= MAZE_X) {
      return false; // Hors labyrinthe
  }

  if (maze[nextY][nextX] == 1) {
      return false; // Mur
  }

  return true;
}

void updatePosition(Robot* robot) {
  switch (robot->direction) {
    case 0: robot->posY++; break;
    case 1: robot->posX++; break;
    case 2: robot->posY--; break;
    case 3: robot->posX--; break;
  }
  Serial.print("Position: (");
  Serial.print(robot->posX);
  Serial.print(", ");
  Serial.print(robot->posY);
  Serial.print(") Direction: ");
  Serial.println(robot->direction);
}

void loop() {
  switch(robot.etat) {
    case 0: // Avancer
      if (canMoveForward(&robot)) {
          avance(&robot, 850);
          updatePosition(&robot);
      } else if (canTurnRight(&robot)) {
          robot.etat = 2; // Tourner à droite si possible
      } else if (canTurnLeft(&robot)) {
          robot.etat = 3; // Tourner à gauche si possible
      } else {
          robot.etat = 1; // Reculer si bloqué
      }
    break;
        
    case 1: // Reculer
      recule(&robot, 300);
      robot.etat = 2; // Tourner à droite après le recul
    break;
        
    case 2: // Tourner à droite
      tourneDroit(&robot);
      if (canMoveForward(&robot)) {
          robot.etat = 0; // Avancer si possible
      } else if (canTurnRight(&robot)) {
          robot.etat = 2; // Tourner à droite si possible
      } else if (canTurnLeft(&robot)) {
          robot.etat = 3; // Tourner à gauche si possible
      } else {
          robot.etat = 1; // Reculer si toujours bloqué
      }
    break;
        
    case 3: // Tourner à gauche
      tourneGauche(&robot);
      if (canMoveForward(&robot)) {
          robot.etat = 0; // Avancer si possible
      } else if (canTurnRight(&robot)) {
          robot.etat = 2; // Tourner à droite si possible
      } else if (canTurnLeft(&robot)) {
          robot.etat = 3; // Tourner à gauche si possible
      } else {
          robot.etat = 1; // Reculer si toujours bloqué
      }
    break;
  }
}