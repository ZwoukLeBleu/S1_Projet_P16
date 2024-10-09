/*
Projet: Parcours robus
Equipe: P16
Auteurs: Arthur, BLAA8729
Description: Fichier source mouvement
Date: 03 octobre 2024
*/


#include <LibRobus.h>
#include <Arduino.h>
#include <Robus\Robus.h>
#include <math.h>
#include "LibMove.h"


extern LIBMOVE_S MainMove;

//struct LibMove_t libMove = { 1.0f, 1.0f, 1.0f};

//LibMove.ConstFR =;


/*void LibRobusInit(int MainMove)
{
    MainMove = MainMove ;
}*/


/*void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON(1);
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}*/

void FullStopNoJump()
{
  //MOTOR_SetSpeed(0,ConstL*0.1);MOTOR_SetSpeed(1,ConstR*0.1);//slow stop
  //delay(10);
  MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);//Full stop
  //vitesse = 0.4;
}

int WheelCalib(uint8_t ForceStatusReset)
{
  static uint8_t status = 0;
  static unsigned long LastTime = 0;
  static int32_t EncoR = 0,EncoL = 0;
  float EncoRF, EncoLF;
  
  if(status == 0)
  {
  LastTime = micros();
  EncoR = abs(ENCODER_Read(RIGHT)), EncoL = abs(ENCODER_Read(LEFT));
  status = 1;
  }
  else if((status == 1) && ((micros()-(200000*MainMove.vitesse) >= LastTime)))
  {
  EncoRF = ((abs(ENCODER_Read(RIGHT)))-EncoR); 
  EncoLF = ((abs(ENCODER_Read(LEFT)))-EncoL);

  
  if(EncoR || EncoR)
    MainMove.ConstRF = (((MainMove.vitesse*EncoLF)/EncoRF)-MainMove.vitesse);
  //ConstLF = (((vitesse*EncoRF)/EncoLF)-0.4);
  //ConstLF = ((200-EncoLF)*0.00005);
  status = 0;
  //Serial.print("EncoLF =:");Serial.println(EncoLF,DEC);
  //Serial.print("ConstRF =:");Serial.println(ConstRF,DEC);
  //Serial.print("ConstLF =:");Serial.println(ConstLF,DEC);
    
  }

  if(ForceStatusReset)
    status = 0;

  return status;
}


/** Fonction to roll the bot foward a set distance
@param dist
Make the bot go foward a set distance in CM

return status
*/
int FowardSetDist(float dist)
{
  uint8_t status = 0;
  //Wheel diameter 85mm, 1 full turn 3200 encoder pusle, wheel circonference = 85mm*3.14159265359
  float WheelCirc = (7.62*PI);
  //WheelDia/ 1 tours = Distance/NbTours . NbTours = Distance/WheelDia
  ENCODER_Reset(0);ENCODER_Reset(1);

  float Nbtour = (3200*(((dist))/WheelCirc)); //1 tours = 3200 pulse

  MOTOR_SetSpeed(LEFT,MainMove.vitesse);
  MOTOR_SetSpeed(RIGHT,(MainMove.ConstRF)+MainMove.vitesse);

  while (1)
  {
    if((ENCODER_Read(0) >= Nbtour) || (ENCODER_Read(1) >= Nbtour))
    {
      break;
    }
    
    /*if(vitesse >= 0.01)
    vitesse -= 0.001;*/
    
    WheelCalib(0);
    MOTOR_SetSpeed(LEFT,(MainMove.ConstLF)+MainMove.vitesse);
    MOTOR_SetSpeed(RIGHT,(MainMove.ConstRF)+MainMove.vitesse);
    //Serial.print("ConstRF :");Serial.println(MainMove.ConstRF,DEC);
  }



  /*float Nbtour = (3200*(((dist*1)/5)/WheelCirc)); //1 tours = 3200 pulse

  vitesse = 0.1;
  MOTOR_SetSpeed(LEFT,(ConstLF)+vitesse);
  MOTOR_SetSpeed(RIGHT,(ConstRF)+vitesse);

  while (1)
  {
    if((ENCODER_Read(0) >= Nbtour) || (ENCODER_Read(1) >= Nbtour))
    {
      break;
    }
    
    if(vitesse <= 0.39)
    vitesse += 0.01;
    
    WheelCalib(0);
    MOTOR_SetSpeed(LEFT,(ConstLF)+vitesse);
    MOTOR_SetSpeed(RIGHT,(ConstRF)+vitesse);
  }

  Nbtour = (3200*(((dist)*(3/5))/WheelCirc)); //1 tours = 3200 pulse
  vitesse = 0.4;
  while (1)
  {
    if((ENCODER_Read(0) >= Nbtour) || (ENCODER_Read(1) >= Nbtour))
    {
      break;
    }
    
    WheelCalib(0);
    MOTOR_SetSpeed(LEFT,(ConstLF)+vitesse);
    MOTOR_SetSpeed(RIGHT,(ConstRF)+vitesse);
  }

  Nbtour = (3200*(((dist*1)/5)/WheelCirc)); //1 tours = 3200 pulse

  while (1)
  {
    if((ENCODER_Read(0) >= Nbtour) || (ENCODER_Read(1) >= Nbtour))
    {
      break;
    }
    
    if(vitesse >= 0.01)
    vitesse -= 0.001;
    
    WheelCalib(0);
    MOTOR_SetSpeed(LEFT,(ConstLF)+vitesse);
    MOTOR_SetSpeed(RIGHT,(ConstRF)+vitesse);
  }*/
  
  
  
  
  FullStopNoJump();
  WheelCalib(1);


  return status;
}

/*int BackwardSetDist(int dist)
{
  uint8_t status = 0;

  //Wheel diameter 85mm, 1 full turn 3200 encoder pusle, wheel circonference = 85mm*3.14159265359
  float WheelCirc = (7.62*PI);

  //WheelDia/ 1 tours = Distance/NbTours . NbTours = Distance/WheelDia

  float Nbtour = (3200*(-dist/WheelCirc)); //1 tours = 3200 pulse

  ENCODER_Reset(0);ENCODER_Reset(1);

  MOTOR_SetSpeed(0,(ConstLF+0.5));
  MOTOR_SetSpeed(1,(ConstRF+0.5));*

  
  //delay(dist/WheelCirc*1000);

  MOTOR_SetSpeed(LEFT,-(ConstLF+vitesse));
  MOTOR_SetSpeed(RIGHT,-(ConstRF+vitesse));
  //delay(dist/WheelCirc*1000);

  while (1)
  {
    if((ENCODER_Read(0) <= Nbtour) || (ENCODER_Read(1) <= Nbtour))
    {
      break;
    }
    WheelCalib(0);
    MOTOR_SetSpeed(LEFT,-(ConstLF+vitesse));
    MOTOR_SetSpeed(RIGHT,-(ConstRF+vitesse));
  }

  FullStopNoJump();
  WheelCalib(1);


  return status;
}*/

void crank90deg(int id)
{
  ENCODER_Reset(1);ENCODER_Reset(0);
  
  if(id == LEFT){
    MOTOR_SetSpeed(RIGHT, -(MainMove.ConstRF+MainMove.vitesse));
    MOTOR_SetSpeed(LEFT, (MainMove.ConstLF+MainMove.vitesse));
  }
  else{
    MOTOR_SetSpeed(RIGHT, (MainMove.ConstRF+MainMove.vitesse));
    MOTOR_SetSpeed(LEFT, -(MainMove.ConstLF+MainMove.vitesse));
  }

  while(1)
  {
   if((ENCODER_Read(id) >= 1900) && (ENCODER_Read(id?LEFT:RIGHT) <= (-1900)))
    {
      break;
    }
    WheelCalib(0);
    if(id == LEFT){
      MOTOR_SetSpeed(RIGHT, -(MainMove.ConstRF+MainMove.vitesse));
      MOTOR_SetSpeed(LEFT, (MainMove.ConstLF+MainMove.vitesse));
    }
    else{
      MOTOR_SetSpeed(RIGHT, (MainMove.ConstRF+MainMove.vitesse));
      MOTOR_SetSpeed(LEFT, -(MainMove.ConstLF+MainMove.vitesse));
    }
  }

 FullStopNoJump();
  WheelCalib(1);
  //Serial.print("Encoder value : ");
  //Serial.println(ENCODER_Read(id),DEC);

  
}

bool Robus_IsBumperALL(void)
{
  if((ROBUS_IsBumper(FRONT))||(ROBUS_IsBumper(LEFT))||(ROBUS_IsBumper(RIGHT))||(ROBUS_IsBumper(REAR)))
    return 1;
  else
    return 0;
}

