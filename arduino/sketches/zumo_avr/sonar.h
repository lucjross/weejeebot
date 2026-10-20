#pragma once

// Sets the sensor pins up. Call once in setup().
void sonarBegin();

// Fires the next pair of ultrasonic sensors (F and R, then B and L) and starts timing their echoes; run it on a timer
// (USS_PING_INTERVAL). Nothing is fired while the bot turns on the spot. Results land in global.uss_distances /
// uss_adj_distances / uss_mm, with uss_distances at -1 while a ping is out.
bool ussPingTimers(void *);
