#include <LibRobus.h>
#include <Arduino.h>

int etat = 0; // 0 = arrêt, 1 = avance, 2 = recule, 3 = tourne à droite, 4 = tourne à gauche
float vitesse = 0.40;
int maze[10][5] = { // 1 = mur, 0 = vide
{1,1,0,1,1},
{1,0,0,0,1},
{1,0,1,0,1},
{1,0,0,0,1},
{1,0,1,0,1},
{1,0,0,0,1},
{1,0,1,0,1},
{1,0,0,0,1},
{1,0,1,0,1},
{1,1,1,1,1}
};

int posX = 3, posY = 1, direction = 0; // Position et direction initiales
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

void recule(int distance) {
  MOTOR_SetSpeed(RIGHT, -vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  delay(distance);
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
  switch(etat) {
    case 0: // Etat arrêt, vérification pour avancer
      if (canMoveForward()) {
        avance(500);
        updatePosition();
        etat = 0;
      } else {
        etat = 1;
      }
      break;
      
    case 1: // Etat recul, obstacle détecté ou mur
      recule(300);
      etat = 2; // Tourner à droite après recul
      break;
      
    case 2: // Etat tourne à droite
      tourneDroit();
      if (!canMoveForward()) {
        etat = 3; // Si toujours bloqué, essaye à gauche
      } else {
        etat = 0; // Sinon continue à avancer
      }
      break;
      
    case 3: // Etat tourne à gauche après échec de tourner à droite
      tourneGauche();
      if (!canMoveForward()) {
        etat = 1; // Si toujours bloqué, revient à reculer
      } else {
        etat = 0; // Sinon continue à avancer
      }
      break;
  }

  delay(200);
}