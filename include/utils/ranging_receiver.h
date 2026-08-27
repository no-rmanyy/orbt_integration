#pragma once
#include <stdint.h>

// Receives per-anchor distances from the three scanning anchors over ESP-NOW,
// runs trilateration, and exposes the robot's computed position.
//
// Call rangingReceiverBegin() once in setup (AFTER WiFi is up, since ESP-NOW
// shares the WiFi radio/channel). Call rangingReceiverUpdate() each loop to
// recompute position when fresh distances are available.

void  rangingReceiverBegin();
void  rangingReceiverUpdate();

// Latest computed position + quality. Valid only if rangingHasFix() is true.
bool  rangingHasFix();
float rangingX();
float rangingY();
float rangingResidual();

// Raw per-anchor distances (metres), for telemetry/debug. -1 = stale/no data.
float rangingDistance(uint8_t anchor_id);   // anchor_id 1..3