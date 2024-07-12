#ifndef STOP_H
#define STOP_H

#include "vector"
#include "AccelStepper.h"
#include "RunningState.h"

using namespace std;

class EmergencyStop{
private:
    bool stopSet;
    bool stopChange; 
    bool stopChangeBlock;
    bool unlockChange; 
    bool unlockChangeBlock;
    
public:
    EmergencyStop();
    bool isStop() {return stopSet;}
    void emergencyChange(const std::vector<AccelStepper*>&);
    RunningState emergencyMove(const vector<AccelStepper*>&);
    void returnChange(const std::vector<AccelStepper*>&);
    RunningState returnMove(const vector<AccelStepper*>&);
    void setStopChange(bool StopChange) { stopChange = StopChange; }
    void setReturnChange(bool UnlockChange) { unlockChange = UnlockChange; }
};

#endif // STOP