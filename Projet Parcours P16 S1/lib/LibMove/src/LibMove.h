/*
Projet S1 2018
Movement libraries for LibRobus
@author Arthur Blanchard
@version 0.1 03/10/2024
*/
#ifndef LibMove_H_
#define LibMove_H_


typedef struct LibMove_t
{
    float ConstRF;
    float ConstLF;
    float vitesse;
}LIBMOVE_S;

void FullStopNoJump();
int FowardSetDist(float dist);
int BackwardSetDist(int dist);
void crank90deg(int id);
bool Robus_IsBumperALL(void);
void LibRobusInit(int MainMove);


#endif //LibMove
