#ifndef MEMORY_H
#define MEMORY_H

#define FILE_NAME "/positions.bin"
#define FILE_NAME_2 "/stages.bin"
#define STAGES_SIZE 24
#define MAX_SIZE_POSITIONS 64
#define MAX_SIZE_STAGES 8

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
    void format(const char*, int, int);
    void writeVector(const char* , int , int, const vector<float>&);
    vector<float> readVector(const char* , int ,int);
};

#endif // MEMORY