/*
Projet: Parcours robus
Equipe: P16
Auteurs: Arthur, BLAA8729
Description: Fichier source mouvement
Date: 03 octobre 2024
*/


#include "LibMove.h"

extern ROBOT OurRobus;

long computePID(long previousTime, float targetSpeed, int motorMaster, int motorSlave);

//Structure locale pour le PID
pidController_t pidController = {0};


// Initialise les paramètres du robot
void initRobot() {
    OurRobus.pos = Center; // Position de départ
    OurRobus.speed = 0.35f; // Vitesse cible
    OurRobus.direction = DROITE; // Direction initiale
    OurRobus.state = SCAN;



}

/*void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON(1);
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}*/


//Fonction qui retourne si 1 des bumpers est active
bool Robus_IsBumperALL(void)
{
  if((ROBUS_IsBumper(FRONT))||(ROBUS_IsBumper(LEFT))||(ROBUS_IsBumper(RIGHT))||(ROBUS_IsBumper(REAR)))
    return 1;
  else
    return 0;
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

// Fonction pour avancer avec régulation PID
void MoveForward() {
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    float targetSpeed = OurRobus.speed;
    unsigned long previousTime = millis();

    float currentDistance = 0.0f;
    //float lastUpdateDistance = 0.0f;

    float speedMaster = 0.0f;
    float speedSlave = 0.0f;

    while (true) {
        
        //int nbrTiles = 0;
        if (currentDistance >= 25.0f) {
            currentDistance -= 25.0f;
                }
        }
        
        if (wallCheck() == 1) { // Mur détecté
            
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
        // delay(50);
}

// Fonction pour tourner à gauche
void TurnLeft(int angle) {

    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, -0.2f); 
    MOTOR_SetSpeed(1, 0.2f); 
    float multi = angle/90;
    while (ENCODER_Read(0) < multi*TURN_PULSES && ENCODER_Read(1) < multi*TURN_PULSES) { }
    
    Stop();

}

// Fonction pour tourner à droite
void TurnRight(int angle) {

    ENCODER_Reset(0);
    ENCODER_Reset(1);

    MOTOR_SetSpeed(0, -0.2f); 
    MOTOR_SetSpeed(1, 0.2f); 
    float multi = angle/90;
    while (ENCODER_Read(0) < multi*TURN_PULSES && ENCODER_Read(1) < multi*TURN_PULSES) { }
    
    Stop();
}

// Fonction pour tourner de 180 degrés
void TurnBack() {
    TurnLeft(180);
}

// Fonction pour exécuter un mouvement
inline void performMovement(Movement movement) {
    delay(15);
    switch (movement.type) {
        case FORWARD:
            MoveForward();
            break;
        case TURN_LEFT:
            TurnLeft(movement.value);
            break;
        case TURN_RIGHT:
            TurnRight(movement.value);
            break;
        case TURN_BACK:
            TurnBack();
            break;
    }
}
