#pragma once

// Declare all autonomous routines
void skills_auton();
void leftAuton();
void rightAuton();
void leftAuton_descore();

// Shakes the bot left/right (~5 degrees) for the given duration in ms.
// Call during unloading to unstick balls.
void shakeBot(int durationMs);