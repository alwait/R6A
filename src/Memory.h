#ifndef MEMORY_H
#define MEMORY_H

#define FILE_NAME "/positions.bin"

#include "SPIFFS.h"
#include "Arduino.h"
#include <vector>
#include "Steering.h"

using namespace std;

class Memory{
private:
    File file;
public:
    Memory();
    void mount();
    void format();
    void writeVector(int, const vector<float>&);
    vector<float> readVector(int);
};

#endif // MEMORY