#include "vector"
#include "Offset.h"
#include "BluetoothConnection.h"
#include <AccelStepper.h>

using namespace std;

class Robot{
private:
    vector<AccelStepper*> steppers;
    vector<Joint*> joints;
    Offset offset;
    BluetoothConnection bluetoothConnection;
    bool running;
    bool enable;
public:
    Robot(vector<AccelStepper*>&, vector<Joint*>&);
    AccelStepper* getStepper(size_t) const;
    Offset& getOffset() {return offset;};
    void moveOffset();
    void emergencyStop();
    void emergencyRelease();
    void moveTestStages();
};