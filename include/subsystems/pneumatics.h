#pragma once

class PneumaticControl {
private:
    bool descoreState;
    bool unloaderState;
    bool R2_lastState;
    bool L2_lastState;

public:
    PneumaticControl();
    void update();
    bool getDescoreState();
    bool getUnloaderState();
};