#pragma once

class OuttakeControl; // Forward declaration

class IntakeControl {
private:
    bool toggleForward;
    bool toggleReverse;
    bool R1_lastState;
    bool A_lastState;

public:
    IntakeControl();
    void update(OuttakeControl& outtake);
    int getVelocity();
    void cancelToggles();
};