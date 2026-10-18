#pragma once
#include "types.h"

// Serial link to the Nano on the planchette.
void linkBegin();

// Bot -> Nano. $arrived(x,y) is sent for every point of interest reached, $smiley() after the last character is shown.
void sendArrived(point p);
void sendSmiley();
void sendCheck();
// Tells the Nano to ask for the planchette to be turned to 0 (after the spin), and that the zero has been recorded.
void sendOrient();
void sendReady();

// Nano -> bot: queues the points from $poi(x,y) messages. Run it on a timer.
bool readNSS(void *);
