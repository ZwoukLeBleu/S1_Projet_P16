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
float vitesse = 0.50;
float ConstR = 0.5;
float ConstL = 0.5;


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
  MOTOR_SetSpeed(0,ConstL*0.1);MOTOR_SetSpeed(1,ConstR*0.1);//slow stop
  delay(10);
  MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);//Full stop
}

int OneWheelTurnBoth()
{
  uint8_t status = 0;

  ENCODER_Reset(1);ENCODER_Reset(0);
  
  MOTOR_SetSpeed(0,(ConstL*vitesse)/2);
  MOTOR_SetSpeed(1,(ConstR*vitesse)/2);
  int32_t EncoR = ENCODER_Read(1), EncoL = ENCODER_Read(0);

  while(1)
  {
    EncoR = ENCODER_Read(1), EncoL = ENCODER_Read(0);
    Serial.print("EncoderR = ");Serial.println(EncoR,DEC);
    Serial.print("EncoderL = ");Serial.println(EncoL,DEC);
    
    if((EncoL >= 800) || (EncoR >= 800))
    {
      break;
    }
  delay(10);
  }
  delay(65);
  MOTOR_SetSpeed(0,0);MOTOR_SetSpeed(1,0);//Full stop
  delay(65);

  MOTOR_SetSpeed(0,-(ConstL*vitesse)/2);
  MOTOR_SetSpeed(1,-(ConstR*vitesse)/2);


  while(1)
  {

    Serial.print("EncoderR = ");Serial.println(ENCODER_Read(1),DEC);
    Serial.print("EncoderL = ");Serial.println(ENCODER_Read(0),DEC);
  
    if((ENCODER_Read(0) <= 0) || (ENCODER_Read(1) <= 0))
    {
      break;
    }
  delay(10);
  }
  delay(65);
  
  FullStopNoJump();

  ConstR += ((800-EncoR)*0.0001);
  ConstL += ((800-EncoL)*0.0001);
  

  return status;
}

int FowardSetDist(int dist)
{
  uint8_t status = 0;

  //Wheel diameter 85mm, 1 full turn 3200 encoder pusle, wheel circonference = 85mm*3.14159265359
  float WheelDia = (8.5*PI);

  //WheelDia/ 1 tours = Distance/NbTours -> NbTours = Distance/WheelDia

  float Nbtour = dist/WheelDia;


  return status;
}

/**void crank90deg(int id)
{
  int i = 0;
  ENCODER_Reset(id);
  
  if(id == 0)
  tourneGauche();
  else
  tourneDroit();

  while(i<(1600))
  {
   i = ENCODER_Read(id);
   delay(10);
    //Serial.println("Encoder value : ");
    //Serial.println(i,DEC);
  }

  arret();
}**/

void setup(){
  BoardInit();
  

  
}

void HalfFowardBoth()
{
  MOTOR_SetSpeed(0,ConstL*vitesse);
  MOTOR_SetSpeed(1,ConstR*vitesse);
}


void loop() {

if(ROBUS_IsBumper(3))
{
  OneWheelTurnBoth();

  HalfFowardBoth();

  delay(2000);
  FullStopNoJump();

}
  
}
