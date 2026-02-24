#pragma once
#include <cstdint>  // For uint32_t

class IntakeControl; // Forward declaration

class OuttakeControl {
private:
    bool comboMode;       // L1 combo: outtake reverse + intake forward
    bool L1_lastState;
    bool midScoringMode;
    bool X_lastState;
    
    uint32_t unjamStartTime;
    bool isUnjamming;

public:
    OuttakeControl();
    void update(IntakeControl& intake);
    int getVelocity();
    bool isMidScoring();
    bool isComboMode();
    void cancelCombo();
};