#pragma once

// Steps to the next of the four ultrasonic sensors and starts its ping; run it on a timer (USS_PING_INTERVAL).
// Results land in global.uss_distances / global.uss_adj_distances, -1 while waiting.
bool ussPingTimers(void *);
