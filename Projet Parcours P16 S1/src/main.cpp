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
#include <math.h>
#include <LibMove.h>

LIBMOVE_S MainMove;


void setup(){
  BoardInit();

  MainMove.ConstLF = 0;
  MainMove.ConstRF = 0;
  MainMove.vitesse = 0.4;

  
  //LibRobusInit(&MainMove);

}


void loop() {
if(ROBUS_IsBumper(REAR))
{
  delay(2000);
  FowardSetDist(100);
  delay(2000);
  FowardSetDist(100);
}
else if (ROBUS_IsBumper(RIGHT))
{
  crank90deg(RIGHT);
}
else if (ROBUS_IsBumper(LEFT))
{
  crank90deg(LEFT);
}
else if (ROBUS_IsBumper(FRONT))
{
  while(1){
  delay(1000);
  FowardSetDist(15);
  delay(1000);
  crank90deg(LEFT);
  delay(500);
  crank90deg(LEFT);
  delay(1000);
  FowardSetDist(15);
  delay(1000);
  crank90deg(LEFT);
  delay(500);
  crank90deg(LEFT);
  if(Robus_IsBumperALL()) break;
  }
  //FullSpeed(2000);


}



    //WheelCalib();
  
}
