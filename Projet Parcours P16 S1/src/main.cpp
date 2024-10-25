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
#include <Adafruit_TCS34725.h>

ROBOT OurRobus;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void InitColorRead(){

if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
}

typedef enum{
RED,
BLUE,
GREEN,
YELLOW,
WHITE
}color_e;


color_e LookColor()
{
    #define ERROR 50

uint16_t clear, red, green, blue;

    tcs.setInterrupt(false);      // turn on LED

    delay(60);  // takes 50ms to read

    tcs.getRawData(&red, &green, &blue, &clear);

    tcs.setInterrupt(true);  // turn off LED

    // Figure out some basic hex code for visualization
    uint32_t sum = clear*0.9;
    float r, g, b;
    r = red; r /= sum;
    g = green; g /= sum;
    b = blue; b /= sum;
    
    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(r);
    Serial.print("\tG:\t"); Serial.print(g);
    Serial.print("\tB:\t"); Serial.print(b);

    
    //r *= 256; g *= 256; b *= 256;
    //Serial.print("\t");
    //Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
    //Serial.println();

    if((r+g)>0.80)
        {return YELLOW;}
    else if(r>0.50)
        {return RED;}
    else if(g>0.39)
        {return GREEN;}
    else if(b>0.53){
        return BLUE;
    }
        
        //Serial.print(" Blue=");Serial.println(b);
    return WHITE;


}


// Configuration initiale
void setup() {
  BoardInit();              // Initialiser le plateau de contrôle OurRobus
    initRobot();              // Initialiser les paramètres du robot
    initSensor();
    InitColorRead();
    
    //while(!analogRead(PIN_5KHZ))
    
    delay(1000);
    
    //while(!whistleCheck()){delay(50);}  
}

// Boucle de fonctionnement principale
void loop() {


switch(LookColor()){
    case RED:
        Serial.println("red");
        break;
    case YELLOW:
        Serial.println("yellow");
        break;
    case BLUE:
        Serial.println("blue");
        break;
    case GREEN:
        Serial.println("green");
        break;
    case WHITE:
        Serial.println("\nBAD_COLOR");
        break;
}

delay(250);
    
    if(Robus_IsBumperALL())
        while(1);

} 