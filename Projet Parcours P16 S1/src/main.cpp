#include <LibRobus.h>
#include <Arduino.h>

int etat = 0; // 0 = arrêt, 1 = avance, 2 = recule, 3 = tourne à droite, 4 = tourne à gauche
float vitesse = 0.40;
int maze[5][5] = { // 1 = mur, 0 = vide
  {1, 1, 1, 1, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 1, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 1, 1, 1, 1}
};

int posX = 1, posY = 1, direction = 0; // Position et direction initiales
float distanceSeuil = 20.0; // Seuil de distance pour détecter un obstacle

void arret() {
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
}

void avance(int distance) {
  MOTOR_SetSpeed(RIGHT, vitesse);
  MOTOR_SetSpeed(LEFT, vitesse);
  delay(distance);
  arret();
}

void recule() {
  MOTOR_SetSpeed(RIGHT, -vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  arret();
}

void tourneDroit() {
  MOTOR_SetSpeed(RIGHT, 0.5 * vitesse);
  MOTOR_SetSpeed(LEFT, -0.5 * vitesse);
  delay(500); // Ajuste en fonction du besoin
  direction = (direction + 1) % 4; // Mise à jour de la direction
}

void tourneGauche() {
  MOTOR_SetSpeed(RIGHT, -0.5 * vitesse);
  MOTOR_SetSpeed(LEFT, 0.5 * vitesse);
  delay(500); // Ajuste en fonction du besoin
  direction = (direction + 3) % 4; // Mise à jour de la direction (retour en arrière)
}

/// @brief Permets de vérifier si le robot peut avancer en fonction des murs et obstacles
/// @return true si le robot peut avancer, false sinon
bool canMoveForward() {
  int nextX = posX, nextY = posY;

  if (direction == 0) {
    nextY++;
  }
  else if (direction == 1) {
    nextX++;
  }
  else if (direction == 2) {
    nextY--;
  }
  else if (direction == 3){
    nextX--;
  }

  if (nextX < 0 || nextX > 4 || nextY < 0 || nextY > 4) {
    return false; // Hors labyrinthe
  }

  return (maze[nextX][nextY] == 0); // Vérifie s'il y a un mur ou non
}

bool isObstacleDetected() {
  float distance = SONAR_GetRange(0);
  if (distance < distanceSeuil) {
    return true;
  }
  return false;
}

void updatePosition() {
  if (direction == 0){
    posY++;
  } 
  else if (direction == 1) {
    posX++;
  }
  else if (direction == 2) {
    posY--;
  }
  else if (direction == 3){
    posX--;
  }
}

void setup() {
  BoardInit();
}

void loop() {
if (etat == 0 && canMoveForward()) {
    if (!isObstacleDetected()) {
      avance(500);
      updatePosition(); 
    } else {
      etat = 2;
    }
  } else {
    recule();
    tourneDroit();
  }

  delay(200);
}