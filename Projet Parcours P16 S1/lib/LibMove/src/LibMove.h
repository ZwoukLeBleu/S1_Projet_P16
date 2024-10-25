// ------------------------------------------------------------------------------------------------
/* BcGE Universite de Sherbrooke- usherbrooke.ca
   COPYRIGHT (c) 2017-2024 BY P16-A24-UdeSherbrooke ALL RIGHTS RESERVED. NO PART OF
   THIS PROGRAM OR PUBLICATION MAY BE REPRODUCED, TRANSMITTED, TRANSCRIBED, STORED IN A RETRIEVAL
   SYSTEM, OR TRANSLATED INTO ANY LANGUAGE OR COMPUTER LANGUAGE IN ANY FORM OR BY ANY MEANS,
   ELECTRONIC, MECHANICAL, MAGNETIC, OPTICAL, CHEMICAL, MANUAL, OR OTHERWISE, WITHOUT THE PRIOR
   WRITTEN PERMISSION OF TGE DEPARTMENT.
*/
// ------------------------------------------------------------------------------------------------
/*!@file
   @brief

   @author Arthur Blanchard
   @version 1.0
*/
// ------------------------------------------------------------------------------------------------

#ifndef LIBMOVE_H_
#define LIBMOVE_H_

#include <LibRobus.h>
#include <Arduino.h>
#include <math.h>

// ================================================================================================
// ================================================================================================
//            DEFINE DECLARATION
// ================================================================================================
// ================================================================================================


#define WHEEL_DIAMETER 7.62f 
#define ENCODER_COUNT 3200
#define PULSE_PER_CM (ENCODER_COUNT / (PI * WHEEL_DIAMETER))
#define TURN_PULSES 1925
#define TURN_VALUE_L 96
#define TURN_VALUE_R 96

// Fonction pour arrêter le robot
#define Stop()  MOTOR_SetSpeed(0, 0);MOTOR_SetSpeed(1, 0)

// ================================================================================================
// ================================================================================================
//            ENUM DECLARATION
// ================================================================================================
// ================================================================================================


// Énumération pour les directions
typedef enum Direction {
    HAUT = 0,
    DROITE = 1,
    BAS = 2,
    GAUCHE = 3
}Direction_e;


//enumeration pour le type de mouvement
typedef enum {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    } MoveType;

// Struct pour représenter un mouvement
typedef struct {
    
    MoveType type;
    int value;
}Movement;




//Enum des positions possible du robot
typedef enum Pos{
    Center,
    White,
    Line,
    GoalColor,
    GoalBlack,
}Position_e;

typedef enum {
    SCAN,
    FOUND,
    CAUGHT,
    RESET,
}State_e;

// ================================================================================================
// ================================================================================================
//            STRUCTURE DECLARATION
// ================================================================================================
// ================================================================================================


// Struct pour représenter le robot
typedef struct{
    float speed;
    Position_e pos;
    Direction_e direction;
    State_e state;
}ROBOT;


// Struct pour l'état du PID
typedef struct PIDController {
    float Kp;
    float Ki;
    unsigned int CT;
} pidController_t;


// ================================================================================================
// ================================================================================================
//            EXTERNAL FUNCTION DECLARATION
// ================================================================================================
// ================================================================================================

bool Robus_IsBumperALL(void);

// Déclaration des fonctions
int wallCheck();
int whistleCheck();
void TurnBack();
void initRobot();
void MoveForward();
void TurnLeft(int angle);
void TurnRight(int angle);
inline void performMovement(Movement movement);



#endif //LibMove
