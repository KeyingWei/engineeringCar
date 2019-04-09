#ifndef __BASEICPERIPHERALS_H
#define __BASEICPERIPHERALS_H

#include "sys.h"

#define LED_GREEN  PFout(14)
#define LED_RED      PEout(11)
#define LASER      PGout(13)
#define LED1   PGout(1)
#define LED2   PGout(2)
#define LED3   PGout(3)
#define LED4   PGout(4)
#define LED5   PGout(5)
#define LED6   PGout(6)
#define LED7   PGout(7)
#define LED8   PGout(8)

#define PUCH1   PFout(0)
#define PUCH2   PFout(1)
#define PUCH3   PEout(4)
#define PUCH4   PEout(5)
#define PUCH5   PEout(6)
#define PUCH6   PEout(12)


#define CHECK1   PCin(0)
#define CHECK2   PCin(1)
#define CHECK3   PCin(2)
#define CHECK4   PCin(3)
#define CHECK5   PCin(4)
#define CHECK6   PCin(5)


#define BEEP_NOTE_1  3814
#define BEEP_NOTE_2  3401
#define BEEP_NOTE_3  3030
#define BEEP_NOTE_4  2865
#define BEEP_NOTE_5  2551
#define BEEP_NOTE_6  2273
#define BEEP_NOTE_7  2024


void BasicPreiph_Init(void);
void StrartingMusic(void);
void FrictionCaliMusic(void);
void FrictionCaliTriggerMusic(void);
void ImuCaliMusic(void);

#endif

