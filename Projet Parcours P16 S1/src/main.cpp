/*
Projet: Imprimante
Equipe: P16
Auteurs: Arthur, BLAA8729
Description: Fichier principale
Date: 25 sept
*/


#include <LibRobus.h>
#include <Arduino.h>
#include <Robus\Robus.h>



bool vert = false;
bool rouge = false;
int etat = 0; // = 0 arrêt 1 = avance 2 = recule 3 = TourneDroit 4 = TourneGauche
int etatPast = 0;
float vitesse = 0.25;
float ConstRF = 0.5;
float ConstLF = 0.5;
float ConstRB = 0.5;
float ConstLB = 0.5;


void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}

void FullStopNoJump()
{
  //MOTOR_SetSpeed(0,ConstL*0.1);MOTOR_SetSpeed(1,ConstR*0.1);//slow stop
  //delay(10);
  MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);//Full stop
}

int WheelCalib()
{
  uint8_t status = 0;

  ENCODER_Reset(1);ENCODER_Reset(0);
  
  MOTOR_SetSpeed(0,(ConstLF*vitesse)/2);
  MOTOR_SetSpeed(1,(ConstRF*vitesse)/2);
  int32_t EncoRF = ENCODER_Read(1), EncoLF = ENCODER_Read(0);

  while(1)
  {
    EncoRF = ENCODER_Read(1), EncoLF = ENCODER_Read(0);
    //Serial.print("EncoderR = ");Serial.println(EncoR,DEC);
    //Serial.print("EncoderL = ");Serial.println(EncoL,DEC);
    
    if((EncoLF >= 800) || (EncoRF >= 800))
    {
      break;
    }
  delay(10);
  }
  delay(65);
  MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);//Full stop
  delay(65);

  MOTOR_SetSpeed(0,-(ConstLB*vitesse)/2);
  MOTOR_SetSpeed(1,-(ConstRB*vitesse)/2);

  while(1)
  {

    //Serial.print("EncoderR = ");Serial.println(ENCODER_Read(1),DEC);
    //Serial.print("EncoderL = ");Serial.println(ENCODER_Read(0),DEC);
  
    if((ENCODER_Read(0) <= 0) || (ENCODER_Read(1) <= 0))
    {
      break;
    }
  }

  int32_t EncoRB = ENCODER_Read(1), EncoLB = ENCODER_Read(0);
  delay(65);
  
  FullStopNoJump();

  ConstRF += ((1600-EncoRF)*0.0001);
  ConstLF += ((1600-EncoLF)*0.0001);

  Serial.print("ConstRF:");Serial.println(ConstRF);
  Serial.print("ConstLF:");Serial.println(ConstLF);
  ConstRB += ((EncoRB)*0.0001);
  ConstLB += ((EncoLB)*0.0001);
  

  return status;
}


/** Fonction to roll the bot foward a set distance
@param dist
Make the bot go foward a set distance in CM


return status
*/
int FowardSetDist(int dist)
{
  uint8_t status = 0;

  //Wheel diameter 85mm, 1 full turn 3200 encoder pusle, wheel circonference = 85mm*3.14159265359
  float WheelCirc = (7.62*PI);

  //WheelDia/ 1 tours = Distance/NbTours -> NbTours = Distance/WheelDia

  float Nbtour = (3200*(dist/WheelCirc)); //1 tours = 3200 pulse

  MOTOR_SetSpeed(0,(ConstLF*vitesse)/2);
  MOTOR_SetSpeed(1,(ConstRF*vitesse)/2);

  ENCODER_Reset(0);ENCODER_Reset(1);

  while (1)
  {
    if((ENCODER_Read(0) >= Nbtour) || (ENCODER_Read(1) >= Nbtour))
    {
      break;
    }

    
    //Serial.print("EncoderR = ");Serial.println(ENCODER_Read(1),DEC);
    //Serial.print("EncoderL = ");Serial.println(ENCODER_Read(0),DEC);

  }

  FullStopNoJump();


  return status;
}

void FullSpeed(int time)
{
  MOTOR_SetSpeed(0,1);
  MOTOR_SetSpeed(1,1);
  delay(time);
  FullStopNoJump();
}

void crank90deg(int id)
{
  ENCODER_Reset(1);ENCODER_Reset(0);
  
  if(id == LEFT)
  {
    MOTOR_SetSpeed(RIGHT, -ConstRF*vitesse);
    MOTOR_SetSpeed(LEFT, ConstLF*vitesse);
  }
  else
  {
    MOTOR_SetSpeed(RIGHT, ConstRF*vitesse);
    MOTOR_SetSpeed(LEFT, -ConstLF*vitesse);
  }

  while(1)
  {
   if((ENCODER_Read(id) >= 1600) || (ENCODER_Read(id?0:1) <= (-1600)))
    {
      break;
    }
  }


  Serial.print("Encoder value : ");
  Serial.println(ENCODER_Read(id),DEC);

  FullStopNoJump();
}

void HalfFowardBoth()
{
  MOTOR_SetSpeed(0,ConstLF*vitesse);
  MOTOR_SetSpeed(1,ConstRF*vitesse);
}

void setup(){
  BoardInit();
  
  while(ROBUS_IsBumper(3) == 0);

WheelCalib();
  
}

void loop() {

if(ROBUS_IsBumper(3))
{
  WheelCalib();


} else if (ROBUS_IsBumper(1))
{
  crank90deg(RIGHT);
}
else if (ROBUS_IsBumper(2))
{
  crank90deg(LEFT);
}
else if (ROBUS_IsBumper(0))
{
  FowardSetDist(50);
}
  
}
