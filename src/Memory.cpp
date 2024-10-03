#include "Memory.h"

Memory::Memory():file(){
}

void Memory::format(){
    SPIFFS.format();
    file = SPIFFS.open(FILE_NAME, "wb");
    if (!file) {Serial.println("Failed to create file"); return;}

    vector<float> emptyData(NUM_OF_AXIS*128, 0.);
    file.write((const uint8_t*)emptyData.data(), NUM_OF_AXIS * sizeof(float) *128); 
    file.close(); 
}


void Memory::mount(){
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS falut");
    } else Serial.println("SPIFFS OK");
}

void Memory::writeVector(int index, const vector<float>& data){
    if(index<0){ Serial.println("Index err"); return;}
    if(data.size() != NUM_OF_AXIS){ Serial.println("Size not correct"); return;}
    file = SPIFFS.open(FILE_NAME, "r+b");
    if(!file){ Serial.println("File open err"); return;}

    int dataPosition = index * NUM_OF_AXIS * sizeof(float);
    file.seek(dataPosition);
    file.write((const uint8_t*)data.data(), NUM_OF_AXIS * sizeof(float));
    file.close();

    Serial.print("Data saved ");
    Serial.print(index);
    Serial.println(":");
    for (size_t i = 0; i < NUM_OF_AXIS; i++){
        Serial.print(data.at(i));
        Serial.print(";");
    }
    Serial.println("");
}

vector<float> Memory::readVector(int index){
    if(index<0){ Serial.println("Index err"); return vector<float>(1,{0});}
    vector<float> data(NUM_OF_AXIS);

    file = SPIFFS.open(FILE_NAME, "rb");
    if(!file){ return data; Serial.println("Read failed");}

    int dataPosition = index * NUM_OF_AXIS * sizeof(float);
    file.seek(dataPosition);
    file.read((uint8_t*)data.data(), NUM_OF_AXIS * sizeof(float));
    file.close();
    
    Serial.print("Data read ");
    Serial.print(index);
    Serial.println(":");
    for (size_t i = 0; i < NUM_OF_AXIS; i++){
        Serial.print(data.at(i));
        Serial.print(";");
    }
    Serial.println("");
    return data;
}