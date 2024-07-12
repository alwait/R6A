#include <AccelStepper.h>
#include <vector>
#include "Joint.h"

using namespace std;

enum RunningState {
    NotRunning,
    Running,
    Stop
};

class Offset{
private:
    bool offsetSet;
    bool offsetSetTemp;
    bool offsetChange; 
    bool offsetChangeBlock;

public:
    Offset();
    bool const& getOffsetSet() const { return offsetSet; }
    bool const& getOffsetSetTemp() const { return offsetSetTemp; }
    bool const& getOffsetChange() const { return offsetChange; }
    bool const& getOffsetChangeBlock() const { return offsetChangeBlock; }
    void setOffsetChange(bool OffsetChange) { offsetChange = OffsetChange; }
    void setOffsetChangeBlock(bool OffsetChangeBlock) { offsetChangeBlock = OffsetChangeBlock; }
    void OffsetChange(const vector<AccelStepper*>&, const vector<Joint*>&);
    RunningState OffsetMove(const vector<AccelStepper*>&);
};
