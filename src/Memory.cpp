#include "Memory.h"

Memory::Memory():file(){
}

void Memory::format(const char* fileName, int dataSize, int savesSize){
    SPIFFS.format();
    file = SPIFFS.open(fileName, "wb");
    if (!file) {Serial.println("Failed to create file"); return;}

    vector<float> emptyData(dataSize*savesSize, 0.);
    file.write((const uint8_t*)emptyData.data(), dataSize * sizeof(float) *savesSize); 
    file.close(); 
}


void Memory::mount(){
    //format(FILE_NAME, NUM_OF_AXIS, MAX_SIZE_POSITIONS);
    //format(FILE_NAME_2, MAX_STAGES_SIZE, MAX_SIZE_POSITIONS);
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS falut");
    } else Serial.println("SPIFFS OK");
}

void Memory::writeVector(const char* fileName, int dataSize, int index, const vector<float>& data){
    if(index<0){ Serial.println("Index err"); return;}
    if(data.size() != dataSize){ Serial.println("Size not correct"); return;}
    file = SPIFFS.open(fileName, "r+b");
    if(!file){ Serial.println("File open err"); return;}

    int dataPosition = index * dataSize * sizeof(float);
    file.seek(dataPosition);
    file.write((const uint8_t*)data.data(), dataSize * sizeof(float));
    file.close();

    Serial.print("Data saved ");
    Serial.print(index);
    Serial.println(":");
    for (size_t i = 0; i < dataSize; i++){
        Serial.print(data.at(i));
        Serial.print(";");
    }
    Serial.println("");
}

vector<float> Memory::readVector(const char* fileName, int dataSize, int index){
    if(index<0){ Serial.println("Index err"); return vector<float>(1,{0});}
    vector<float> data(dataSize);

    file = SPIFFS.open(fileName, "rb");
    if(!file){ return data; Serial.println("Read failed");}

    int dataPosition = index * dataSize * sizeof(float);
    file.seek(dataPosition);
    file.read((uint8_t*)data.data(), dataSize * sizeof(float));
    file.close();
    
    Serial.print("Data read ");
    Serial.print(index);
    Serial.println(":");
    for (size_t i = 0; i < dataSize; i++){
        Serial.print(data.at(i));
        Serial.print(";");
    }
    Serial.println("");
    return data;
}