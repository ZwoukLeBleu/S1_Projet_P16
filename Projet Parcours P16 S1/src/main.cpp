/*
Projet: Imprimante
Equipe: P16
Auteurs: Arthur, BLAA8729
Description: Fichier principale
Date: 25 sept
*/


#include <Arduino.h>
#include <stdbool.h>

#include <math.h>

#include <LibRobus.h>
#include <LibSensor.h>
#include <LibMove.h>


#define SERIAL_DEBUG 1

ROBOT OurRobus;




  
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









// Initialise les paramètres du robot
void initRobot() {
    OurRobus.pos = Center; // Position de départ
    OurRobus.speed = 0.35f; // Vitesse cible
    OurRobus.direction = DROITE; // Direction initiale
    OurRobus.state = SCAN;
}



// Configuration initiale
void setup() {
  BoardInit();              // Initialiser le plateau de contrôle OurRobus
    initRobot();              // Initialiser les paramètres du robot
    while(!analogRead(PIN_5KHZ));

    delay(1000);
    
    while(!whistleCheck()){delay(100);}  
}

// Boucle de fonctionnement principale
void loop() {



}